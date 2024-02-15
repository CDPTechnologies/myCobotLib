CDPVERSION = 4.12
TYPE = library
PROJECTNAME = myCobotLib

DEPS += \

HEADERS += \
    myCobotCommander.h \
    myCobotIO.h \
    mycobotlib.h \
    myCobotLibBuilder.h

SOURCES += \
    myCobotCommander.cpp \
    myCobotIO.cpp \
    myCobotLibBuilder.cpp

DISTFILES += $$files(*.xml, true) \
    Templates/Models/myCobotLib.myCobotIO.xml

load(cdp)
