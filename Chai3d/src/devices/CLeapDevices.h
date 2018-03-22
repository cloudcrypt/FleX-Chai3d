//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CLeapDevicesH
#define CLeapDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_LEAP_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CLeapDevices.h

    \brief
    Implements support for the Leap Motion tracker interface.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cLeapDevice;
typedef std::shared_ptr<cLeapDevice> cLeapDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cLeapDevice
    \ingroup    devices

    \brief
    This class implements an interface to the Leap Motion tracker
    interface.

    \details
    This class implements an interface to the Leap Motion tracker
    interface.
*/
//==============================================================================
class cLeapDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cLeapDevice.
    cLeapDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cLeapDevice.
    virtual ~cLeapDevice();

    //! Shared cPhantomDevice allocator.
    static cLeapDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cLeapDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

 public:

    //! This method opens a connection to the device.
    virtual bool open();

    //! This method closes the connection to the device.
    virtual bool close();

    //! This method calibrates the device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of haptic devices available for this class of devices.
    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! This method opens the libraries for this class of devices.
    static bool openLibraries();

    //! This method closes the libraries for this class of devices.
    static bool closeLibraries();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DEVICE LIBRARIES:
    //--------------------------------------------------------------------------

protected:

    //! Number of instances for this class of devices currently using the libraries.
    static unsigned int s_libraryCounter;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS
    //--------------------------------------------------------------------------

protected:

    //! Update data from controllers.
    bool updateData();
};

//------------------------------------------------------------------------------
} // namespace chai3ds
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //_DISABLE_LEAP_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
