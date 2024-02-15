#include "myCobotIO.h"
#include <OSAPI/Process/OSAPISemaphore.h>
#include <IO/ServerIO/DeltaValidatorSendTrigger.h>
#include <exception>
#include <cmath>
#include <cstddef>

using namespace myCobotLib;
using namespace CDP::System::Base;
using namespace std;

enum class ServoControlMode {
  Released = 0,
  PositionControl = 1,
  SpeedControl = 2,
  PositionSpeedControl = 3,
};

enum class GripperControlMode {
  NoControl = 0,
  ReadState = 1,
  SetAndReadState = 2
};

/*!
  \class myCobotLib::myCobotIO
  \inmodule myCobotLib

  \section1 Usage

  A Custom IOServer that can send/receive data using a Transport class.
*/

myCobotIO::myCobotIO()
{
}

myCobotIO::~myCobotIO()
{
  if (m_transport)
  {
    delete m_transport;
    m_transport = nullptr;
  }
}

void myCobotIO::Create(const char* fullName)
{
  IOServer::Create(fullName);
  m_commDebugFlags.Create(this, "CommDebugFlags", e_PropertySaveOnChange);
  m_channelManager.reset(new ServerIO::ChannelManager(this));
}

void myCobotIO::CreateModel()
{
  IOServer::CreateModel();
}

void myCobotIO::Configure(const char* componentXML)
{
  IOServer::Configure(componentXML);
  m_commander.SetDebugPrefix(GetNodeLongName());
  m_servoControlMode.SetChangedFlag(true);
}

void myCobotIO::Activate()
{
  IOServer::Activate();
  // Start the Main() thread with name of the component and normal priority:
  Start(CDPTHREAD_PRIORITY_NORMAL,ShortName());
}

void myCobotIO::Suspend()
{
  Stop();           // set Stop flag
  m_event.Set();    // set event so that Wait in Main() completes.
  Stop(true);       // set Stop flag, block
  Delete();         // will block until thread is no longer running (for max. 2 seconds)
  IOServer::Suspend();
}

void myCobotIO::Destroy()
{
  for (auto channel : m_channelManager->GetChannelList())
    m_channelManager->DeregisterCDPChannel(channel);
}

bool myCobotIO::IsCommProblem()
{
  return (CDPTime::GetGlobalTime()-m_lastUpdateTime>=m_transport->GetTimeout());
}

void myCobotIO::FillNodeChildren(CDP::StudioAPI::NodeStream &serializer) const
{
  IOServer::FillNodeChildren(serializer);
  serializer << CDP::StudioAPI::AdoptedChild(m_transport);
  serializer << m_atomLedColorRed << m_atomLedColorGreen << m_atomLedColorBlue;
  serializer << m_servoControlMode << m_servoMode1SpeedLimit;
  serializer << m_joint1AngleCurrent << m_joint2AngleCurrent << m_joint3AngleCurrent
             << m_joint4AngleCurrent << m_joint5AngleCurrent << m_joint6AngleCurrent;
  serializer << m_joint1AngleDesired << m_joint2AngleDesired << m_joint3AngleDesired
             << m_joint4AngleDesired << m_joint5AngleDesired << m_joint6AngleDesired;
  serializer << m_joint1SpeedDesired << m_joint2SpeedDesired << m_joint3SpeedDesired
             << m_joint4SpeedDesired << m_joint5SpeedDesired << m_joint6SpeedDesired;
  serializer << m_getActuatorPosition
             << m_actuatorX << m_actuatorY << m_actuatorZ
             << m_actuatorAngleX << m_actuatorAngleY << m_actuatorAngleZ;
  serializer << m_gripperControlMode << m_gripperSpeedLimit << m_gripperOpennessDesired << m_gripperOpennessCurrent;
  serializer << m_controlAtom << m_atomButtonState;
}

ServerIO::ChannelManager* myCobotIO::GetChannelManager() const
{
  return m_channelManager.get();
}

/*!
  brief Main thread function, runs asynchronously from state-machine.
*/
void myCobotIO::Main()
{
  while (!Stopped())
  {
    m_event.Wait();
    m_event.Reset();
    m_commander.SetDebugFlags(m_commDebugFlags);
    if (Stopped())
      break;

    if (m_transport->IsError())
      m_transport->Close();
    if (!m_transport->IsOpen())
    {
      if (m_transport->Open(CDP::IO::Transport::OpenMode_Send)==false && DebugLevel(DEBUGLEVEL_NORMAL))
      {
        CDPMessage("ERROR: %s: Failed opening transport. Please check connection and Transport configuration.\n", GetNodeLongName().c_str());
        OSAPISleep(100);
      }
    }
    else if (!IsCommProblem() && !IsState("Online"))
      RunInComponentThread([&]{ requestedState="Online"; });

    ProtocolImplementation();
  }
  m_transport->Close();
}

void myCobotIO::ProtocolImplementation()
{
  m_lastUpdateTime = CDPTime::GetGlobalTime();

  m_channelManager->SynchronizeValuesIn();

  auto angles = m_commander.GetAngles();
  if (angles.has_value())
  {
    m_joint1AngleCurrent.SetSyncedInValue(angles.value().joint1);
    m_joint2AngleCurrent.SetSyncedInValue(angles.value().joint2);
    m_joint3AngleCurrent.SetSyncedInValue(angles.value().joint3);
    m_joint4AngleCurrent.SetSyncedInValue(angles.value().joint4);
    m_joint5AngleCurrent.SetSyncedInValue(angles.value().joint5);
    m_joint6AngleCurrent.SetSyncedInValue(angles.value().joint6);
  }

  if (m_getActuatorPosition.GetSyncedInValue())
  {
    if (auto position = m_commander.GetActuatorPosition(); position.has_value())
    {
      m_actuatorX.SetSyncedInValue(position.value().x);
      m_actuatorY.SetSyncedInValue(position.value().y);
      m_actuatorZ.SetSyncedInValue(position.value().z);
      m_actuatorAngleX.SetSyncedInValue(position.value().angleX);
      m_actuatorAngleY.SetSyncedInValue(position.value().angleY);
      m_actuatorAngleZ.SetSyncedInValue(position.value().angleX);
    }
  }

  auto servoControlMode = m_servoControlMode.GetSyncedInValue();
  if (servoControlMode == static_cast<unsigned char>(ServoControlMode::PositionControl))
  {
    if (m_servoControlMode.IsChanged()
        || m_joint1AngleDesired.IsChanged() || m_joint2AngleDesired.IsChanged() || m_joint3AngleDesired.IsChanged()
        || m_joint4AngleDesired.IsChanged() || m_joint5AngleDesired.IsChanged() || m_joint6AngleDesired.IsChanged())
    /*
    if (fabs((float)(m_joint1AngleDesired.GetSyncedInValue() - m_joint1AngleCurrent.GetSyncedInValue())) > 0.0001745
        ||
        fabs((float)(m_joint2AngleDesired.GetSyncedInValue() - m_joint2AngleCurrent.GetSyncedInValue())) > 0.0001745
        ||
        fabs((float)(m_joint3AngleDesired.GetSyncedInValue() - m_joint3AngleCurrent.GetSyncedInValue())) > 0.0001745
        ||
        fabs((float)(m_joint4AngleDesired.GetSyncedInValue() - m_joint4AngleCurrent.GetSyncedInValue())) > 0.0001745
        ||
        fabs((float)(m_joint5AngleDesired.GetSyncedInValue() - m_joint5AngleCurrent.GetSyncedInValue())) > 0.0001745
        ||
        fabs((float)(m_joint6AngleDesired.GetSyncedInValue() - m_joint6AngleCurrent.GetSyncedInValue())) > 0.0001745)
        */
    {
      myCobotCommander::JointAnglesRad angles;
      angles.joint1 = m_joint1AngleDesired.GetSyncedInValue();
      angles.joint2 = m_joint2AngleDesired.GetSyncedInValue();
      angles.joint3 = m_joint3AngleDesired.GetSyncedInValue();
      angles.joint4 = m_joint4AngleDesired.GetSyncedInValue();
      angles.joint5 = m_joint5AngleDesired.GetSyncedInValue();
      angles.joint6 = m_joint6AngleDesired.GetSyncedInValue();
      m_commander.SetAngles(angles, m_servoMode1SpeedLimit.GetSyncedInValue());
    }
    m_joint1AngleDesired.SetChangedFlag(false);
    m_joint2AngleDesired.SetChangedFlag(false);
    m_joint3AngleDesired.SetChangedFlag(false);
    m_joint4AngleDesired.SetChangedFlag(false);
    m_joint5AngleDesired.SetChangedFlag(false);
    m_joint6AngleDesired.SetChangedFlag(false);
  }
  else if (servoControlMode == static_cast<unsigned char>(ServoControlMode::SpeedControl))
  {
    if (m_joint1SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(1, m_joint1SpeedDesired.GetSyncedInValue());
      m_joint1SpeedDesired.SetChangedFlag(false);
    }
    if (m_joint2SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(2, m_joint2SpeedDesired.GetSyncedInValue());
      m_joint2SpeedDesired.SetChangedFlag(false);
    }
    if (m_joint3SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(3, m_joint3SpeedDesired.GetSyncedInValue());
      m_joint3SpeedDesired.SetChangedFlag(false);
    }
    if (m_joint4SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(4, m_joint4SpeedDesired.GetSyncedInValue());
      m_joint4SpeedDesired.SetChangedFlag(false);
    }
    if (m_joint5SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(5, m_joint5SpeedDesired.GetSyncedInValue());
      m_joint5SpeedDesired.SetChangedFlag(false);
    }
    if (m_joint6SpeedDesired.IsChanged())
    {
      m_commander.SetSpeed(6, m_joint6SpeedDesired.GetSyncedInValue());
      m_joint6SpeedDesired.SetChangedFlag(false);
    // }
    }
  }
  else if (servoControlMode == static_cast<unsigned char>(ServoControlMode::PositionSpeedControl))
  {
    if (m_servoControlMode.IsChanged()
        || m_joint1AngleDesired.IsChanged() || m_joint2AngleDesired.IsChanged() || m_joint3AngleDesired.IsChanged()
        || m_joint4AngleDesired.IsChanged() || m_joint5AngleDesired.IsChanged() || m_joint6AngleDesired.IsChanged()
        || m_joint1SpeedDesired.IsChanged() || m_joint2SpeedDesired.IsChanged() || m_joint3SpeedDesired.IsChanged()
        || m_joint4SpeedDesired.IsChanged() || m_joint5SpeedDesired.IsChanged() || m_joint6SpeedDesired.IsChanged())
    {
      myCobotCommander::JointAnglesRad angles;
      angles.joint1 = m_joint1AngleDesired.GetSyncedInValue();
      angles.joint2 = m_joint2AngleDesired.GetSyncedInValue();
      angles.joint3 = m_joint3AngleDesired.GetSyncedInValue();
      angles.joint4 = m_joint4AngleDesired.GetSyncedInValue();
      angles.joint5 = m_joint5AngleDesired.GetSyncedInValue();
      angles.joint6 = m_joint6AngleDesired.GetSyncedInValue();
      myCobotCommander::JointSpeedsRadS speeds;
      speeds.joint1 = m_joint1SpeedDesired.GetSyncedInValue();
      speeds.joint2 = m_joint2SpeedDesired.GetSyncedInValue();
      speeds.joint3 = m_joint3SpeedDesired.GetSyncedInValue();
      speeds.joint4 = m_joint4SpeedDesired.GetSyncedInValue();
      speeds.joint5 = m_joint5SpeedDesired.GetSyncedInValue();
      speeds.joint6 = m_joint6SpeedDesired.GetSyncedInValue();
      m_commander.SetAngles(angles, speeds);
    }
    m_joint1AngleDesired.SetChangedFlag(false);
    m_joint2AngleDesired.SetChangedFlag(false);
    m_joint3AngleDesired.SetChangedFlag(false);
    m_joint4AngleDesired.SetChangedFlag(false);
    m_joint5AngleDesired.SetChangedFlag(false);
    m_joint6AngleDesired.SetChangedFlag(false);
    m_joint1SpeedDesired.SetChangedFlag(false);
    m_joint2SpeedDesired.SetChangedFlag(false);
    m_joint3SpeedDesired.SetChangedFlag(false);
    m_joint4SpeedDesired.SetChangedFlag(false);
    m_joint5SpeedDesired.SetChangedFlag(false);
    m_joint6SpeedDesired.SetChangedFlag(false);
  }
  else
    if (m_servoControlMode.IsChanged())
      m_commander.SetReleaseServos();
  if (m_servoControlMode.IsChanged())
    m_servoControlMode.SetChangedFlag(false);

  if (m_controlAtom.GetSyncedInValue())
  {
    if (m_controlAtom.IsChanged() ||
        m_atomLedColorRed.IsChanged() || m_atomLedColorGreen.IsChanged() || m_atomLedColorBlue.IsChanged())
    {
      m_commander.SetColor(m_atomLedColorRed.GetSyncedInValue(),
                           m_atomLedColorGreen.GetSyncedInValue(),
                           m_atomLedColorBlue.GetSyncedInValue());
      m_atomLedColorRed.SetChangedFlag(false);
      m_atomLedColorGreen.SetChangedFlag(false);
      m_atomLedColorBlue.SetChangedFlag(false);
    }
    if (auto state = m_commander.GetAtomButonState(); state.has_value())
      m_atomButtonState.SetSyncedInValue(!state.value());
  }
  if (m_controlAtom.IsChanged())
    m_controlAtom.SetChangedFlag(false);

  auto gripperControlMode = m_gripperControlMode.GetSyncedInValue();
  if (gripperControlMode == static_cast<unsigned char>(GripperControlMode::ReadState) ||
      gripperControlMode == static_cast<unsigned char>(GripperControlMode::SetAndReadState))
  {
    if (auto openness = m_commander.GetGripperOpenness(); openness.has_value())
      m_gripperOpennessCurrent.SetSyncedInValue(openness.value());
  }
  if (gripperControlMode == static_cast<unsigned char>(GripperControlMode::SetAndReadState))
  {
    if (m_gripperOpennessDesired.IsChanged())
      m_commander.SetGripperOpenness(m_gripperOpennessDesired.GetSyncedInValue(), m_gripperSpeedLimit.GetSyncedInValue());
    m_gripperOpennessDesired.SetChangedFlag(false);
  }
  if (m_gripperControlMode.IsChanged())
    m_gripperControlMode.SetChangedFlag(false);

  m_channelManager->SynchronizeValuesOut();
}

bool myCobotIO::HandleXMLElement(XMLElementEx *xml)
{
  if (xml->GetName()=="Transport")
  {
    m_transport = CDP::IO::Transport::Create(xml,this);
    if (m_transport==nullptr)
      Suspend();
    else
      m_transport->Configure(xml,this);
    m_commander.SetTransport(m_transport);
    return true;
  }
  if (xml->GetName()=="IOChannel")
  {
    string name = xml->GetAttributeValue("Name");
    if (name == "ServoControlMode")
      m_servoControlMode.CreateConfigure(xml, this);
    else if (name == "ServoMode1SpeedLimit")
      m_servoMode1SpeedLimit.CreateConfigure(xml, this);
    else if (name == "Joint1AngleCurrent")
      m_joint1AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint2AngleCurrent")
      m_joint2AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint3AngleCurrent")
      m_joint3AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint4AngleCurrent")
      m_joint4AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint5AngleCurrent")
      m_joint5AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint6AngleCurrent")
      m_joint6AngleCurrent.CreateConfigure(xml, this);
    else if (name == "Joint1AngleDesired")
      m_joint1AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint2AngleDesired")
      m_joint2AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint3AngleDesired")
      m_joint3AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint4AngleDesired")
      m_joint4AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint5AngleDesired")
      m_joint5AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint6AngleDesired")
      m_joint6AngleDesired.CreateConfigure(xml, this);
    else if (name == "Joint1SpeedDesired")
      m_joint1SpeedDesired.CreateConfigure(xml, this);
    else if (name == "Joint2SpeedDesired")
      m_joint2SpeedDesired.CreateConfigure(xml, this);
    else if (name == "Joint3SpeedDesired")
      m_joint3SpeedDesired.CreateConfigure(xml, this);
    else if (name == "Joint4SpeedDesired")
      m_joint4SpeedDesired.CreateConfigure(xml, this);
    else if (name == "Joint5SpeedDesired")
      m_joint5SpeedDesired.CreateConfigure(xml, this);
    else if (name == "Joint6SpeedDesired")
      m_joint6SpeedDesired.CreateConfigure(xml, this);
    else if (name == "GetActuatorPosition")
      m_getActuatorPosition.CreateConfigure(xml, this);
    else if (name == "ActuatorX")
      m_actuatorX.CreateConfigure(xml, this);
    else if (name == "ActuatorY")
      m_actuatorY.CreateConfigure(xml, this);
    else if (name == "ActuatorZ")
      m_actuatorZ.CreateConfigure(xml, this);
    else if (name == "ActuatorAngleX")
      m_actuatorAngleX.CreateConfigure(xml, this);
    else if (name == "ActuatorAngleY")
      m_actuatorAngleY.CreateConfigure(xml, this);
    else if (name == "ActuatorAngleZ")
      m_actuatorAngleZ.CreateConfigure(xml, this);
    else if (name == "GripperControlMode")
      m_gripperControlMode.CreateConfigure(xml, this);
    else if (name == "GripperSpeedLimit")
      m_gripperSpeedLimit.CreateConfigure(xml, this);
    else if (name == "GripperOpennessDesired")
      m_gripperOpennessDesired.CreateConfigure(xml, this);
    else if (name == "GripperOpennessCurrent")
      m_gripperOpennessCurrent.CreateConfigure(xml, this);
    else if (name == "ControlAtom")
      m_controlAtom.CreateConfigure(xml, this);
    else if (name == "AtomLedColorRed")
      m_atomLedColorRed.CreateConfigure(xml, this);
    else if (name == "AtomLedColorGreen")
      m_atomLedColorGreen.CreateConfigure(xml, this);
    else if (name == "AtomLedColorBlue")
      m_atomLedColorBlue.CreateConfigure(xml, this);
    else if (name == "AtomButtonState")
      m_atomButtonState.CreateConfigure(xml, this);
  }
  return IOServer::HandleXMLElement(xml); // call base
}

void myCobotIO::ProcessOffline()
{
  IOServer::ProcessOffline();
  m_event.Set(); // this triggers the main thread
}

void myCobotIO::ProcessOnline()
{
  IOServer::ProcessOnline();
  m_event.Set(); // this triggers the main thread
}

template <typename T>
void myCobotIOChannel<T>::CreateConfigure(XMLElementEx* xml, myCobotIO* parent)
{
  string name = xml->GetAttributeValue("Name");
  string input = xml->GetAttributeValue("Input");
  int dummyModule = 0;
  int dummyGroup = 0;
  ServerIO::CDPSignalChannel<T>::Create(name.c_str(), parent);
  ServerIO::CDPSignalChannel<T>::SetChannelParameters(input == "1", dummyModule, dummyGroup, 0);
  ServerIO::CDPSignalChannel<T>::Configure(xml);
  parent->GetChannelManager()->SetSyncGroupId(input == "1", this);
  parent->GetChannelManager()->RegisterCDPChannel(this);
  ServerIO::CDPSignalChannel<T>::RegisterValidator(new ServerIO::DeltaValidatorSendTrigger(this));
  ServerIO::CDPSignalChannel<T>::SetDataPointer(&m_syncedInValue);
}

template <typename T>
T myCobotIOChannel<T>::GetSyncedInValue()
{
  return m_syncedInValue;
}

template <typename T>
void myCobotIOChannel<T>::SetSyncedInValue(T val)
{
  m_syncedInValue = val;
}

template <typename T>
bool myCobotIOChannel<T>::IsChanged() const
{
  return m_isChanged;
}

template <typename T>
void myCobotIOChannel<T>::SetChangedFlag(bool isChanged)
{
  m_isChanged = isChanged;
}

template<typename T>
void myCobotIOChannel<T>::FlagForSend()
{
  m_isChanged = true;
}
