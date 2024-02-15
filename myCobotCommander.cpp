#include "myCobotCommander.h"
#include <OSAPI/Process/OSAPIProcess.h>

using namespace myCobotLib;
using namespace std;

namespace {

const byte COMMAND_PREFIX{0xFE};
const byte COMMAND_SUFFIX{0xFA};

const unsigned int MYCOBOT_NO_FEEDBACK_COMMAND_DELAY_MS{100};
const unsigned int MYCOBOT_WITH_FEEDBACK_COMMAND_DELAY_MS{50};

const unsigned char ATOM_BUTTON_GPIO_PIN_NR{39};

vector<double> LINK_RAD_TO_PERCENTAGE_SPEED_COEFFICENTS = {
  100,
  100,
  100,
  100,
  100,
  100
};

vector<double> LINK_MAX_ALLOWED_JOINT_ANGLE_RAD = {
  2.87979,
  2,
  2,
  2,
  2,
  3.05
};

vector<double> LINK_MIN_ALLOWED_JOINT_ANGLE_RAD = {
  -2.87979,
  -2,
  -2,
  -2,
  -2,
  -3.05
};

enum class CommmDebugFlags {
  ServoSet = 1,
  ServoGet = 1 << 1,
  GripperSet = 1 << 2,
  GripperGet = 1 << 3,
  AtomSet = 1 << 4,
  AtomGet = 1 << 5,
  BinaryDump = 1 << 6,
};

double radiansToDegrees(double radians)
{
  const double conversionFactor = 180.0 / PI;
  return radians * conversionFactor;
}

double degreesToRadians(double degrees)
{
  const double conversionFactor = 180.0 / PI;
  return degrees / conversionFactor;
}

double radiansToEncoder(double radians)
{
  const double conversionFactor = 2048.0 / PI;
  return max(min(4096.0, 2048.0 - (radians * conversionFactor)), 0.0);
}

double speedToEncoder(double speedRadS)
{
  const double conversionFactor = 180.0 / PI;
  return max(min(abs(speedRadS) * conversionFactor / 150.0 * 1700.0, 1700.0), 1.0);
}

byte* intPtrToByteArray(int16_t* value)
{
  return reinterpret_cast<byte*>(value);
}

int16_t byteArrayToInt(byte byteLow, byte byteHigh)
{
  int result{0};
  auto resultBytes = reinterpret_cast<byte*>(&result);
  resultBytes[1] = byteLow;
  resultBytes[0] = byteHigh;
  return result;
}

}

void myCobotCommander::SetTransport(CDP::IO::Transport* transport)
{
  m_transport = transport;
}

void myCobotCommander::SetDebugPrefix(const string& prefix)
{
  m_debugPrefix = prefix;
}

void myCobotCommander::SetDebugFlags(unsigned char flags)
{
  m_debugFlags = flags;
}

void myCobotCommander::SetColor(unsigned char r, unsigned char g, unsigned char b)
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::AtomSet))
    CDPMessage("COMMDEBUG: %s: Sending command SetColor(R=%hhd,G=%hhd,G=%hhd)\n", m_debugPrefix.c_str(),
               r, g, b);
  SendCommand(SerialCommands::SetColor, {byte{r}, byte{g}, byte{b}});
  OSAPISleep(MYCOBOT_NO_FEEDBACK_COMMAND_DELAY_MS);
}

void myCobotCommander::SetAngle(unsigned char linkId, double angleRad, double speedRadS)
{
  int16_t aDegx100 = radiansToDegrees(angleRad) * 100;
  unsigned char speedPercentage = min((unsigned char)abs(LINK_RAD_TO_PERCENTAGE_SPEED_COEFFICENTS[linkId-1] * speedRadS),
      (unsigned char)100);
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoSet))
    CDPMessage("COMMDEBUG: %s: Sending command SetAngle(Link=%hhd,AngleRad=%lf(%hd),Speed=%lf(%hhd)\n",
               m_debugPrefix.c_str(), linkId, angleRad, aDegx100, speedRadS, speedPercentage);
  auto aDegx100Bytes = intPtrToByteArray(&aDegx100);
  SendCommand(SerialCommands::SendAngle, {byte{linkId}, aDegx100Bytes[1], aDegx100Bytes[0], byte{speedPercentage}});
  OSAPISleep(MYCOBOT_WITH_FEEDBACK_COMMAND_DELAY_MS);
}

void myCobotCommander::SetReleaseServos()
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoSet))
    CDPMessage("COMMDEBUG: %s: Sending command ReleaseServos()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::ReleaseAllServos, {});
  OSAPISleep(MYCOBOT_NO_FEEDBACK_COMMAND_DELAY_MS);
}

void myCobotCommander::JointBrake(unsigned char linkId)
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoSet))
    CDPMessage("COMMDEBUG: %s: Sending command JointBrake(Link=%hhd)\n", m_debugPrefix.c_str(),
               linkId);
  SendCommand(SerialCommands::JointBrake, {byte{linkId}});
  OSAPISleep(MYCOBOT_NO_FEEDBACK_COMMAND_DELAY_MS);
}

void myCobotCommander::SetSpeed(unsigned char linkId, double speedRadS)
{
  unsigned char speedPercentage = min((unsigned char)abs(LINK_RAD_TO_PERCENTAGE_SPEED_COEFFICENTS[linkId-1] * speedRadS),
      (unsigned char)100);
  if (speedPercentage == 0)
    JointBrake(linkId);
  else
    SetAngle(linkId,
             speedRadS > 0 ? LINK_MAX_ALLOWED_JOINT_ANGLE_RAD[linkId-1] : LINK_MIN_ALLOWED_JOINT_ANGLE_RAD[linkId-1],
             speedPercentage);
}

void myCobotCommander::SetAngles(const JointAnglesRad& angles, unsigned char speedPercentage)
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoSet))
    CDPMessage("COMMDEBUG: %s: Sending command SetAngles(J1=%lf,J2=%lf,J3=%lf,J4=%lf,J5=%lf,J6=%lf,speed=%hhd)\n",
               m_debugPrefix.c_str(), angles.joint1, angles.joint2, angles.joint3, angles.joint4, angles.joint5,
               angles.joint6, speedPercentage);
  int16_t j1Degx100 = radiansToDegrees(angles.joint1) * 100;
  int16_t j2Degx100 = radiansToDegrees(angles.joint2) * 100;
  int16_t j3Degx100 = radiansToDegrees(angles.joint3) * 100;
  int16_t j4Degx100 = radiansToDegrees(angles.joint4) * 100;
  int16_t j5Degx100 = radiansToDegrees(angles.joint5) * 100;
  int16_t j6Degx100 = radiansToDegrees(angles.joint6) * 100;
  auto j1Degx100Bytes = intPtrToByteArray(&j1Degx100);
  auto j2Degx100Bytes = intPtrToByteArray(&j2Degx100);
  auto j3Degx100Bytes = intPtrToByteArray(&j3Degx100);
  auto j4Degx100Bytes = intPtrToByteArray(&j4Degx100);
  auto j5Degx100Bytes = intPtrToByteArray(&j5Degx100);
  auto j6Degx100Bytes = intPtrToByteArray(&j6Degx100);
  SendCommand(SerialCommands::SendAngles, {j1Degx100Bytes[1], j1Degx100Bytes[0],
                                            j2Degx100Bytes[1], j2Degx100Bytes[0],
                                            j3Degx100Bytes[1], j3Degx100Bytes[0],
                                            j4Degx100Bytes[1], j4Degx100Bytes[0],
                                            j5Degx100Bytes[1], j5Degx100Bytes[0],
                                            j6Degx100Bytes[1], j6Degx100Bytes[0],
                                            byte{speedPercentage}});
  OSAPISleep(MYCOBOT_WITH_FEEDBACK_COMMAND_DELAY_MS);
}

void myCobotCommander::SetAngles(const JointAnglesRad& angles, const JointSpeedsRadS& speeds)
{
  int16_t j1AngleEnc = radiansToEncoder(angles.joint1);
  int16_t j2AngleEnc = radiansToEncoder(angles.joint2);
  int16_t j3AngleEnc = radiansToEncoder(-angles.joint3);
  int16_t j4AngleEnc = radiansToEncoder(angles.joint4);
  int16_t j5AngleEnc = radiansToEncoder(angles.joint5);
  int16_t j6AngleEnc = radiansToEncoder(angles.joint6);
  int16_t j1SpeedEnc = speedToEncoder(speeds.joint1);
  int16_t j2SpeedEnc = speedToEncoder(speeds.joint2);
  int16_t j3SpeedEnc = speedToEncoder(speeds.joint3);
  int16_t j4SpeedEnc = speedToEncoder(speeds.joint4);
  int16_t j5SpeedEnc = speedToEncoder(speeds.joint5);
  int16_t j6SpeedEnc = speedToEncoder(speeds.joint6);
  if (j1AngleEnc != m_lastj1AngleEnc || j2AngleEnc != m_lastj2AngleEnc || j3AngleEnc != m_lastj3AngleEnc
      || j4AngleEnc != m_lastj4AngleEnc || j5AngleEnc != m_lastj5AngleEnc || j6AngleEnc != m_lastj6AngleEnc
      || j1SpeedEnc != m_lastj1SpeedEnc || j2SpeedEnc != m_lastj2SpeedEnc || j3SpeedEnc != m_lastj3SpeedEnc
      || j4SpeedEnc != m_lastj4SpeedEnc || j5SpeedEnc != m_lastj5SpeedEnc || j6SpeedEnc != m_lastj6SpeedEnc)
  {
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoSet))
      CDPMessage("COMMDEBUG: %s: Sending command SetAngles(J1=%lf(s=%lf),J2=%lf(s=%lf),J3=%lf(s=%lf),J4=%lf(s=%lf),J5=%lf(s=%lf),J6=%lf(s=%lf))\n",
                 m_debugPrefix.c_str(),
                 angles.joint1, speeds.joint1,
                 angles.joint2, speeds.joint2,
                 angles.joint3, speeds.joint3,
                 angles.joint4, speeds.joint4,
                 angles.joint5, speeds.joint5,
                 angles.joint6, speeds.joint6);
    auto j1AngleEncBytes = intPtrToByteArray(&j1AngleEnc);
    auto j2AngleEncBytes = intPtrToByteArray(&j2AngleEnc);
    auto j3AngleEncBytes = intPtrToByteArray(&j3AngleEnc);
    auto j4AngleEncBytes = intPtrToByteArray(&j4AngleEnc);
    auto j5AngleEncBytes = intPtrToByteArray(&j5AngleEnc);
    auto j6AngleEncBytes = intPtrToByteArray(&j6AngleEnc);
    auto j1SpeedEncBytes = intPtrToByteArray(&j1SpeedEnc);
    auto j2SpeedEncBytes = intPtrToByteArray(&j2SpeedEnc);
    auto j3SpeedEncBytes = intPtrToByteArray(&j3SpeedEnc);
    auto j4SpeedEncBytes = intPtrToByteArray(&j4SpeedEnc);
    auto j5SpeedEncBytes = intPtrToByteArray(&j5SpeedEnc);
    auto j6SpeedEncBytes = intPtrToByteArray(&j6SpeedEnc);
    SendCommand(SerialCommands::SetEncodersDrag, {j1AngleEncBytes[1], j1AngleEncBytes[0],
                                                  j2AngleEncBytes[1], j2AngleEncBytes[0],
                                                  j3AngleEncBytes[1], j3AngleEncBytes[0],
                                                  j4AngleEncBytes[1], j4AngleEncBytes[0],
                                                  j5AngleEncBytes[1], j5AngleEncBytes[0],
                                                  j6AngleEncBytes[1], j6AngleEncBytes[0],
                                                  j1SpeedEncBytes[1], j1SpeedEncBytes[0],
                                                  j2SpeedEncBytes[1], j2SpeedEncBytes[0],
                                                  j3SpeedEncBytes[1], j3SpeedEncBytes[0],
                                                  j4SpeedEncBytes[1], j4SpeedEncBytes[0],
                                                  j5SpeedEncBytes[1], j5SpeedEncBytes[0],
                                                  j6SpeedEncBytes[1], j6SpeedEncBytes[0]});
    m_lastj1AngleEnc = j1AngleEnc;
    m_lastj2AngleEnc = j2AngleEnc;
    m_lastj3AngleEnc = j3AngleEnc;
    m_lastj4AngleEnc = j4AngleEnc;
    m_lastj5AngleEnc = j5AngleEnc;
    m_lastj6AngleEnc = j6AngleEnc;
    m_lastj1SpeedEnc = j1SpeedEnc;
    m_lastj2SpeedEnc = j2SpeedEnc;
    m_lastj3SpeedEnc = j3SpeedEnc;
    m_lastj4SpeedEnc = j4SpeedEnc;
    m_lastj5SpeedEnc = j5SpeedEnc;
    m_lastj6SpeedEnc = j6SpeedEnc;
    OSAPISleep(MYCOBOT_WITH_FEEDBACK_COMMAND_DELAY_MS);
  }
}

std::optional<myCobotCommander::JointAnglesRad> myCobotCommander::GetAngles()
{
  JointAnglesRad angles;
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
    CDPMessage("COMMDEBUG: %s: Sending command GetAngles()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::GetAngles, {});
  if (auto response = ReceiveCommandResponse(SerialCommands::GetAngles); response.size() == 12)
  {
    int16_t j1aDegx100 = byteArrayToInt(response[0], response[1]);
    int16_t j2aDegx100 = byteArrayToInt(response[2], response[3]);
    int16_t j3aDegx100 = byteArrayToInt(response[4], response[5]);
    int16_t j4aDegx100 = byteArrayToInt(response[6], response[7]);
    int16_t j5aDegx100 = byteArrayToInt(response[8], response[9]);
    int16_t j6aDegx100 = byteArrayToInt(response[10], response[11]);
    angles.joint1 = degreesToRadians(j1aDegx100/100.0);
    angles.joint2 = degreesToRadians(j2aDegx100/100.0);
    angles.joint3 = degreesToRadians(j3aDegx100/100.0);
    angles.joint4 = degreesToRadians(j4aDegx100/100.0);
    angles.joint5 = degreesToRadians(j5aDegx100/100.0);
    angles.joint6 = degreesToRadians(j6aDegx100/100.0);
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
      CDPMessage("COMMDEBUG: %s: Received command GetAngles() response J1=%lf,J2=%lf,J3=%lf,J4=%lf,J5=%lf,J6=%lf\n",
                 m_debugPrefix.c_str(),
                 angles.joint1, angles.joint2, angles.joint3,
                 angles.joint4, angles.joint5, angles.joint6);
    return angles;
  }
  return {};
}

std::optional<myCobotCommander::ActuatorPosition> myCobotCommander::GetActuatorPosition()
{
  ActuatorPosition position;
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
    CDPMessage("COMMDEBUG: %s: Sending command GetCoords()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::GetCoords, {});
  if (auto response = ReceiveCommandResponse(SerialCommands::GetCoords); response.size() == 12)
  {
    int16_t x10 = byteArrayToInt(response[0], response[1]);
    int16_t y10 = byteArrayToInt(response[2], response[3]);
    int16_t z10 = byteArrayToInt(response[4], response[5]);
    int16_t ax100 = byteArrayToInt(response[6], response[7]);
    int16_t ay100 = byteArrayToInt(response[8], response[9]);
    int16_t az100 = byteArrayToInt(response[10], response[11]);
    position.x = x10/10.0;
    position.y = y10/10.0;
    position.z = z10/10.0;
    position.angleX = degreesToRadians(ax100/100.0);
    position.angleY = degreesToRadians(ay100/100.0);
    position.angleZ = degreesToRadians(az100/100.0);
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
      CDPMessage("COMMDEBUG: %s: Received command GetCoords() response X=%lf,Y=%lf,Z=%lf,aX=%lf,aY=%lf,aZ=%lf\n",
                 m_debugPrefix.c_str(),
                 position.x, position.y, position.z,
                 position.angleX, position.angleY, position.angleZ);
    return position;
  }
  return {};
}

void myCobotCommander::SetGripperOpenness(unsigned char openness, unsigned char speed)
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::GripperSet))
    CDPMessage("COMMDEBUG: %s: Sending command SetGripperValue(Openness=%hhd,Speed=%hhd)\n", m_debugPrefix.c_str(),
               openness, speed);
  SendCommand(SerialCommands::SetGripperValue, {byte{openness}, byte{speed}});
  OSAPISleep(MYCOBOT_WITH_FEEDBACK_COMMAND_DELAY_MS);
}

optional<unsigned char> myCobotCommander::GetGripperOpenness()
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::GripperGet))
    CDPMessage("COMMDEBUG: %s: Sending command GetGripperValue()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::GetGripperValue, {});
  if (auto response = ReceiveCommandResponse(SerialCommands::GetGripperValue); response.size() == 1)
  {
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::GripperGet))
      CDPMessage("COMMDEBUG: %s: Received command GetGripperValue() response Openness=%hhd\n", m_debugPrefix.c_str(),
                 response[0]);
    return (unsigned char)response[0];
  }
  return {};
}

std::optional<bool> myCobotCommander::GetDigitalInput(unsigned char gpioPinNumber)
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::AtomGet))
    CDPMessage("COMMDEBUG: %s: Sending command GetDigitalInput()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::GetDigitalInput, {byte{gpioPinNumber}});
  if (auto response = ReceiveCommandResponse(SerialCommands::GetDigitalInput); response.size() == 2)
  {
    if (response[0] == byte{gpioPinNumber})
    {
      if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::AtomGet))
        CDPMessage("COMMDEBUG: %s: Received command GetDigitalInput() response Level=%hhd\n", m_debugPrefix.c_str(),
                   response[1]);
      return (bool)response[1];
    }
  }
  return {};
}

std::optional<bool> myCobotCommander::GetAtomButonState()
{
  return GetDigitalInput(ATOM_BUTTON_GPIO_PIN_NR);
}

optional<unsigned char> myCobotCommander::GetPowerStatus()
{
  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
    CDPMessage("COMMDEBUG: %s: Sending command GetPowerStatus()\n", m_debugPrefix.c_str());
  SendCommand(SerialCommands::IsPowerOn, {});
  if (auto response = ReceiveCommandResponse(SerialCommands::IsPowerOn); response.size() == 1)
  {
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::ServoGet))
      CDPMessage("COMMDEBUG: %s: Received command GetPowerStatus() response Status=%hhd\n", m_debugPrefix.c_str(),
                 response[0]);
    return (unsigned char)response[0];
  }
  return {};
}

void myCobotCommander::SendCommand(SerialCommands command, vector<byte> commandData)
{
  if (m_transport)
  {
    vector<byte> sendData;
    sendData.emplace_back(COMMAND_PREFIX);
    sendData.emplace_back(COMMAND_PREFIX);
    unsigned char len = commandData.size() + 2;
    sendData.emplace_back(byte{len});
    sendData.emplace_back(byte{command});
    sendData.insert(sendData.end(), commandData.begin(), commandData.end());
    sendData.emplace_back(COMMAND_SUFFIX);
    if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::BinaryDump))
      CDPMessage("COMMDEBUG: %s: Sending command 0x%02hhx with data len %d\n", m_debugPrefix.c_str(), command, commandData.size());
    m_transport->Write(reinterpret_cast<const char*>(sendData.data()), sendData.size());
  }
  else
    CDPMessage("COMMERROR: %s: Can not send command 0x%02hhx - transport not set\n", m_debugPrefix.c_str(), command);
}

std::vector<byte> myCobotCommander::ReceiveCommandResponse(SerialCommands command)
{
  vector<byte> response;
  if (m_transport)
  {
    byte readDataByte[1];
    unsigned short skippedBytes{0};
    auto readLen = m_transport->Read(reinterpret_cast<char*>(readDataByte), 1);
    while (readLen > 0)
    {
      if (readDataByte[0] == COMMAND_PREFIX)
      {
        readLen = m_transport->Read(reinterpret_cast<char*>(readDataByte), 1);
        if (readLen > 0 && readDataByte[0] == COMMAND_PREFIX)
        {
          byte lenByte[1];
          readLen = m_transport->Read(reinterpret_cast<char*>(lenByte), 1);
          if (readLen > 0)
          {
            unsigned char len = (unsigned char)lenByte[0];
            vector<byte> receivedData;
            receivedData.resize(len);
            readLen = m_transport->Read(reinterpret_cast<char*>(receivedData.data()), len);
            if (readLen == len)
            {
              unsigned char dataIndex = 0;
              if (receivedData[dataIndex] == byte{command})
              {
                response.reserve(len);
                for (++dataIndex; dataIndex < len - 1; ++dataIndex)
                  response.emplace_back(receivedData[dataIndex]);
                if (receivedData[dataIndex] == COMMAND_SUFFIX)
                {
                  if (m_debugFlags & static_cast<unsigned char>(CommmDebugFlags::BinaryDump))
                    CDPMessage("COMMDEBUG: %s: Received command 0x%02hhx response with length %d (skipped %hu bytes)\n",
                               m_debugPrefix.c_str(), command, response.size(), skippedBytes);
                  return response;
                }
              }
            }
          }
        }
        ++skippedBytes;
      }
      else
      {
        readLen = m_transport->Read(reinterpret_cast<char*>(readDataByte), 1);
        ++skippedBytes;
      }
    }
    CDPMessage("COMMERROR: %s: Failed to receive command 0x%02hhx response (received=%u bytes)\n", m_debugPrefix.c_str(),
               command, readLen);
    return {};
  }
  else
    CDPMessage("COMMERROR: %s: Can not send command 0x%02hhx - transport not set\n", m_debugPrefix.c_str(), command);
  return response;
}
