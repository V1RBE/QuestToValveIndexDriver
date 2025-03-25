#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include "openvr.h"
#include "driver.h"

//using namespace vr;  // Removed the global using directive

// We need this constant, but it isn't in the header for some reason.
//extern const uint32_t k_unMaxTrackedDeviceCount = 16;  // Removed extern, defined here
namespace vr {
    const uint32_t kInvalidTrackedDeviceIndex = 0xFFFFFFFF;
}
// ------------------------------------------------------------------------------------------------
// MyController
// ------------------------------------------------------------------------------------------------
class MyController : public vr::IVRRenderController
{
public:
    MyController()
        : m_unControllerId(vr::kInvalidTrackedDeviceIndex)
    {
        //m_vrState = {}; // Initialize in Activate
    }

    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override
    {
        m_unControllerId = unObjectId;
        m_vrState = {}; // Initialize here
        std::cout << "Controller Activated: " << m_unControllerId << std::endl;

        // Set up the VR properties (manufacturer, model number, etc.)
        vr::IVRDriverHost* pDriverHost = vr::VRDriver::GetGenericInterface(vr::IVRDriverHost_Version);
        if (pDriverHost) {
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_WillDriftInYaw_Bool, false);
            pDriverHost->TrackedDevicePropertyString(m_unControllerId, vr::Prop_ManufacturerName_String, "My Manufacturer");
            pDriverHost->TrackedDevicePropertyString(m_unControllerId, vr::Prop_ModelNumber_String, "My Controller Model");
            pDriverHost->TrackedDevicePropertyInt32(m_unControllerId, vr::Prop_ControllerRole_Int32, vr::TrackedControllerRole_T::TrackedControllerRole_LeftHand); // Or RightHand
            pDriverHost->TrackedDevicePropertyUint64(m_unControllerId, vr::Prop_TrackedDeviceClass_Uint64, vr::TrackedDeviceClass_Controller);

            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_HasControllerTracking_Bool, true);
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_Input_HasProximitySensor_Bool, false);
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_Input_HasTouchPad_Bool, false);
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_Input_HasJoystick_Bool, false);
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_Input_HasTrigger_Bool, true);
            pDriverHost->TrackedDevicePropertyBool(m_unControllerId, vr::Prop_Input_HasGrip_Bool, true);
        }
        else {
            std::cout << "Failed to get IVRDriverHost in Activate" << std::endl;
            return vr::VRInitError_InvalidParam;
        }

        return vr::VRInitError_None;
    }

    virtual void Deactivate() override
    {
        m_unControllerId = vr::kInvalidTrackedDeviceIndex;
        std::cout << "Controller Deactivated" << std::endl;
    }

    virtual void RunFrame() override
    {
        if (m_unControllerId == vr::kInvalidTrackedDeviceIndex)
            return;

        // In a real driver, you would update the pose here from real tracking data.
        // For our dummy driver, we'll just simulate some movement.
        static float fPositionX = 0.0f;
        static float fPositionY = 0.0f;
        static float fPositionZ = -2.0f;
        static float fRotationY = 0.0f;

        fPositionX += 0.01f;
        fPositionY = sin(fPositionX) * 0.1f;
        fRotationY += 1.0f;

        // Populate the DriverPose_t struct.  (Corrected struct name)
        vr::DriverPose_t pose = { 0 };  //  Initialize the struct to zero
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        // Position
        pose.vecPosition[0] = fPositionX;
        pose.vecPosition[1] = fPositionY;
        pose.vecPosition[2] = fPositionZ;

        // Velocity and Acceleration (linear and angular) - set to zero for this example.
        memset(pose.vecVelocity, 0, sizeof(pose.vecVelocity));
        memset(pose.vecAcceleration, 0, sizeof(pose.vecAcceleration));
        memset(pose.vecAngularVelocity, 0, sizeof(pose.vecAngularVelocity));
        memset(pose.vecAngularAcceleration, 0, sizeof(pose.vecAngularAcceleration));

        // Rotation (Quaternion)
        // Convert the Y rotation to a quaternion.  Simple rotation around Y axis.
        float fRadians = fRotationY * (3.14159f / 180.0f);
        pose.qRotation.w = cos(fRadians / 2.0f);
        pose.qRotation.x = 0.0f;
        pose.qRotation.y = sin(fRadians / 2.0f);
        pose.qRotation.z = 0.0f;

        pose.qWorldFromDriverRotation = pose.qRotation;
        pose.qDriverFromHeadRotation.w = 1.0f;

        vr::IVRDriverHost* pDriverHost = vr::VRDriver::GetGenericInterface(vr::IVRDriverHost_Version);
        if (pDriverHost)
        {
            pDriverHost->TrackedDevicePoseUpdated(m_unControllerId, pose, sizeof(pose));
        }
        else {
            std::cout << "Failed to get IVRDriverHost in RunFrame" << std::endl;
        }
    }

    virtual void GetInputState(vr::VRControllerState_t* pState) override
    {
        *pState = m_vrState;
    }

    virtual vr::EVRInputError GetControllerState(vr::VRControllerState_t* pControllerState) override
    {
        *pControllerState = m_vrState;
        return vr::VRInputError_None;
    }

    virtual void TriggerHapticPulse(uint32_t unAxisId, uint16_t usDurationMicroSec) override
    {
        std::cout << "TriggerHapticPulse" << std::endl;
        // Do nothing for now.
    }
    virtual void AttachToDisplay(const vr::DriverTransform_t& pose, vr::ETrackingUniverseOrigin origin) override
    {
        //  Not implemented
    }

    virtual uint32_t GetId() const override {
        return m_unControllerId;
    }

private:
    vr::TrackedDeviceIndex_t m_unControllerId;
    vr::VRControllerState_t m_vrState;
};

// ------------------------------------------------------------------------------------------------
// MyDriverContext
// ------------------------------------------------------------------------------------------------
class MyDriverContext : public vr::IVRDriverContext
{
public:
    MyDriverContext()
        : m_pDriverHost(nullptr),
        m_pLeftController(nullptr),
        m_pRightController(nullptr),
        m_unLeftControllerId(vr::kInvalidTrackedDeviceIndex),
        m_unRightControllerId(vr::kInvalidTrackedDeviceIndex)
    {
    }

    virtual vr::EVRInitError Init(vr::IVRDriverHost* pDriverHost, const char* pchLogOutput) override
    {
        m_pDriverHost = pDriverHost;
        if (m_pDriverHost == nullptr)
            return vr::VRInitError_InvalidParam;

        // Initiazlie OpenVR
        if (VR_INIT_SERVER_DRIVER_CONTEXT(pDriverHost) != vr::VRInitError_None)
        {
            m_pDriverHost = nullptr;
            return vr::VRInitError_Init_Internal; // Or appropriate error
        }

        m_pLeftController = new MyController();
        m_pRightController = new MyController();
        if (!m_pLeftController || !m_pRightController) {
            Cleanup();
            return vr::VRInitError_OutOfMemory;
        }

        return vr::VRInitError_None;
    }

    virtual void Cleanup() override
    {
        if (m_pLeftController)
        {
            // m_pLeftController->Deactivate(); // commented out because MyController doesn't have a Deactivate method.
            delete m_pLeftController;
            m_pLeftController = nullptr;
        }
        if (m_pRightController)
        {
            //m_pRightController->Deactivate(); // commented out because MyController doesn't have a Deactivate method.
            delete m_pRightController;
            m_pRightController = nullptr;
        }
        m_pDriverHost = nullptr;
        VR_SHUTDOWN_SERVER_DRIVER_CONTEXT();
    }

    virtual const char* GetInterfaceVersion() override
    {
        return vr::IVRDriverContext_Version;
    }

    virtual vr::IVRTrackedDeviceDriver* GetTrackedDeviceDriver(uint32_t unWhich) override
    {

        if (unWhich == m_unLeftControllerId)
        {
            return m_pLeftController;
        }
        else if (unWhich == m_unRightControllerId)
        {
            return m_pRightController;
        }
        return nullptr;
    }
    virtual vr::IVRTrackedDeviceProvider* GetTrackedDeviceProvider() override
    {
        return this;
    }

    virtual vr::IVRRenderController* GetRenderController() override
    {
        return nullptr; // This driver doesn't provide any rendering.
    }

    virtual void* GetGenericInterface(const char* pchInterfaceName) override
    {
        return nullptr;
    }

    virtual vr::EVRInitError GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override
    {
        if (pnWidth)
            *pnWidth = 1024;
        if (pnHeight)
            *pnHeight = 1024;
        return vr::VRInitError_None;
    }

    virtual void RunFrame() override
    {
        if (m_pLeftController)
        {
            m_pLeftController->RunFrame();
        }
        if (m_pRightController)
        {
            m_pRightController->RunFrame();
        }
    }

    virtual bool ShouldBlockStandbyMode() override
    {
        return false;
    }

    virtual void EnterStandby() override
    {
    }

    virtual void LeaveStandby() override
    {
    }

    virtual uint32_t GetTrackedDeviceCount() override
    {
        return 2; // Left and right controllers.
    }

    virtual vr::TrackedDeviceIndex_t TrackedDeviceIdToDriver(uint32_t unId) override
    {
        if (unId == 0)
            return m_unLeftControllerId;
        else if (unId == 1)
            return m_unRightControllerId;
        return vr::kInvalidTrackedDeviceIndex;
    }

    void DisconnectDevice(uint32_t unId)  // Added DisconnectDevice
    {
        if (unId == m_unLeftControllerId)
        {
            std::cout << "Disconnecting Left Controller" << std::endl;
            m_unLeftControllerId = vr::kInvalidTrackedDeviceIndex;
            m_pLeftController = nullptr; //  Set the pointer to null
        }
        else if (unId == m_unRightControllerId)
        {
            std::cout << "Disconnecting Right Controller" << std::endl;
            m_unRightControllerId = vr::kInvalidTrackedDeviceIndex;
            m_pRightController = nullptr; // Set the pointer to null
        }
    }

private:
    vr::IVRDriverHost* m_pDriverHost;
    MyController* m_pLeftController;
    MyController* m_pRightController;
    vr::TrackedDeviceIndex_t m_unLeftControllerId;
    vr::TrackedDeviceIndex_t m_unRightControllerId;
};

// ------------------------------------------------------------------------------------------------
// Exported Functions
// ------------------------------------------------------------------------------------------------
MyDriverContext g_DriverContext;

//Removed EXPORT_API
vr::IVRDriverContext* VR_CALLTYPE HmdDriverFactory(int interfaceVersion)
{
    if (interfaceVersion == vr::IVRDriverContext_Version)
    {
        return &g_DriverContext;
    }

    return nullptr;
}
