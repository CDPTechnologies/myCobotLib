#ifndef MYCOBOTLIB_MYCOBOTIO_H
#define MYCOBOTLIB_MYCOBOTIO_H

#include "myCobotCommander.h"
#include <CDPSystem/Base/CDPComponent.h>
#include <CDPSystem/Base/ThreadSafeProperty.h>
#include <Signal/CDPSignal.h>
#include <CDPParameter/CDPParameter.h>
#include <CDPAlarm/CDPAlarm.h>
#include <OSAPI/Process/OSAPIThread.h>
#include <OSAPI/Process/OSAPIEvent.h>
#include <IO/Transport.h>
#include <CDPSystem/Base/CDPProperty.h>
#include <IO/IOServer.h>
#include <IO/ServerIO/CDPSignalChannel.h>
#include <IO/ServerIO/ChannelManager.h>
#include <IO/ServerIO/ISendTrigger.h>

#include <atomic>
#include <memory>

namespace myCobotLib {

class myCobotIO;

template <typename T>
class myCobotIOChannel : public ServerIO::CDPSignalChannel<T>, public ServerIO::ISendTrigger
{
public:
  void CreateConfigure(XMLElementEx *xml, myCobotIO* parent);
  T GetSyncedInValue();
  void SetSyncedInValue(T val);
  bool IsChanged() const;
  void SetChangedFlag(bool isChanged);

private:
  void FlagForSend() override;
  T m_syncedInValue{};
  bool m_isChanged{false};
};

class myCobotIO : public IOServer, public OSAPIThread
{
public:
  myCobotIO();
  ~myCobotIO() override;

  void Create(const char* fullName) override;
  void CreateModel() override;
  void Configure(const char* componentXML) override;
  void Activate() override;
  void Suspend() override;
  void Destroy() override;
  bool IsCommProblem() override;
  void FillNodeChildren(CDP::StudioAPI::NodeStream &serializer) const override;
  ServerIO::ChannelManager* GetChannelManager() const;

protected:
  void ProcessOffline() override;
  void ProcessOnline() override;

  bool HandleXMLElement(XMLElementEx *pEx) override;
  void Main() override;

  virtual void ProtocolImplementation();

  using CDPComponent::requestedState;
  using CDPComponent::ts;
  using CDPComponent::fs;
  CDP::IO::Transport* m_transport{nullptr};
  OSAPIEvent m_event;
  std::atomic<double> m_lastUpdateTime{0};

  myCobotIOChannel<unsigned char> m_servoControlMode;
  myCobotIOChannel<unsigned char> m_servoMode1SpeedLimit;
  myCobotIOChannel<double> m_joint1AngleDesired;
  myCobotIOChannel<double> m_joint2AngleDesired;
  myCobotIOChannel<double> m_joint3AngleDesired;
  myCobotIOChannel<double> m_joint4AngleDesired;
  myCobotIOChannel<double> m_joint5AngleDesired;
  myCobotIOChannel<double> m_joint6AngleDesired;
  myCobotIOChannel<double> m_joint1SpeedDesired;
  myCobotIOChannel<double> m_joint2SpeedDesired;
  myCobotIOChannel<double> m_joint3SpeedDesired;
  myCobotIOChannel<double> m_joint4SpeedDesired;
  myCobotIOChannel<double> m_joint5SpeedDesired;
  myCobotIOChannel<double> m_joint6SpeedDesired;
  myCobotIOChannel<double> m_joint1AngleCurrent;
  myCobotIOChannel<double> m_joint2AngleCurrent;
  myCobotIOChannel<double> m_joint3AngleCurrent;
  myCobotIOChannel<double> m_joint4AngleCurrent;
  myCobotIOChannel<double> m_joint5AngleCurrent;
  myCobotIOChannel<double> m_joint6AngleCurrent;
  myCobotIOChannel<bool> m_getActuatorPosition;
  myCobotIOChannel<double> m_actuatorX;
  myCobotIOChannel<double> m_actuatorY;
  myCobotIOChannel<double> m_actuatorZ;
  myCobotIOChannel<double> m_actuatorAngleX;
  myCobotIOChannel<double> m_actuatorAngleY;
  myCobotIOChannel<double> m_actuatorAngleZ;
  myCobotIOChannel<unsigned char> m_gripperControlMode;
  myCobotIOChannel<unsigned char> m_gripperSpeedLimit;
  myCobotIOChannel<unsigned char> m_gripperOpennessDesired;
  myCobotIOChannel<unsigned char> m_gripperOpennessCurrent;
  myCobotIOChannel<bool> m_controlAtom;
  myCobotIOChannel<unsigned char> m_atomLedColorRed;
  myCobotIOChannel<unsigned char> m_atomLedColorGreen;
  myCobotIOChannel<unsigned char> m_atomLedColorBlue;
  myCobotIOChannel<bool> m_atomButtonState;

  std::unique_ptr<ServerIO::ChannelManager> m_channelManager;
  myCobotCommander m_commander;

  CDP::System::Base::ThreadSafeProperty<unsigned char> m_commDebugFlags;
};

} // namespace myCobotLib

#endif
