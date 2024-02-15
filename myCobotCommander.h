#ifndef MYCOBOTLIB_MYCOBOTCOMMANDER_H
#define MYCOBOTLIB_MYCOBOTCOMMANDER_H

#include <IO/Transport.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace myCobotLib {

enum class SerialCommands : unsigned char {
  IsPowerOn = 0x12,
  ReleaseAllServos = 0x13,
  GetAngles = 0x20,
  SendAngle = 0x21,
  SendAngles = 0x22,
  GetCoords = 0x23,
  SetEncodersDrag = 0x3E,
  JointBrake = 0x55,
  GetDigitalInput = 0x62,
  GetGripperValue = 0x65,
  SetGripperValue = 0x67,
  SetColor= 0x6A
};

class myCobotCommander {
public:
  void SetTransport(CDP::IO::Transport* transport);
  void SetDebugPrefix(const std::string& prefix);
  void SetDebugFlags(unsigned char flags);

  std::optional<unsigned char> GetPowerStatus();
  void SetColor(unsigned char r, unsigned char g, unsigned char b);
  void SetAngle(unsigned char linkId, double angleRad, double speedRadS);
  void SetReleaseServos();
  void JointBrake(unsigned char linkId);

  void SetSpeed(unsigned char linkId, double speedRadS);

  struct JointAnglesRad {
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;
    double joint6;
  };
  struct JointSpeedsRadS {
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;
    double joint6;
  };
  void SetAngles(const JointAnglesRad& angles, unsigned char speedPercentage);
  void SetAngles(const JointAnglesRad& angles, const JointSpeedsRadS& speeds);
  std::optional<JointAnglesRad> GetAngles();

  struct ActuatorPosition {
    double x;
    double y;
    double z;
    double angleX;
    double angleY;
    double angleZ;
  };
  std::optional<ActuatorPosition> GetActuatorPosition();

  void SetGripperOpenness(unsigned char openness, unsigned char speed);
  std::optional<unsigned char> GetGripperOpenness();

  std::optional<bool> GetDigitalInput(unsigned char gpioPinNumber);
  std::optional<bool> GetAtomButonState();

private:
  CDP::IO::Transport* m_transport;
  unsigned char m_debugFlags;
  std::string m_debugPrefix;
  void SendCommand(SerialCommands command, std::vector<std::byte> commandData);
  std::vector<std::byte> ReceiveCommandResponse(SerialCommands command);
  int16_t m_lastj1AngleEnc{0};
  int16_t m_lastj2AngleEnc{0};
  int16_t m_lastj3AngleEnc{0};
  int16_t m_lastj4AngleEnc{0};
  int16_t m_lastj5AngleEnc{0};
  int16_t m_lastj6AngleEnc{0};
  int16_t m_lastj1SpeedEnc{0};
  int16_t m_lastj2SpeedEnc{0};
  int16_t m_lastj3SpeedEnc{0};
  int16_t m_lastj4SpeedEnc{0};
  int16_t m_lastj5SpeedEnc{0};
  int16_t m_lastj6SpeedEnc{0};
};

}

#endif
