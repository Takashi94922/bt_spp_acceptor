//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;

    void computeAngles();
    void updateBodyQuaternion();

    //-------------------------------------------------------------------------------------------
    // Function declarations
public:
    float qDot1, qDot2, qDot3, qDot4;
    float qb0 = 1.0f, qb1 = 0.0f, qb2 = 0.0f, qb3 = 0.0f; // 機体座標の姿勢（q ⊗ q_mount）
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    
    //IMUの取り付け角度に対するクオータニオン（V字）
    //機体x=IMU z、機体y=IMU x、機体z=IMU y
    static constexpr float imw = 0.5f;
    static constexpr float imx = -0.5f;
    static constexpr float imy = -0.5f;
    static constexpr float imz = -0.5f;

    Madgwick();
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void setBeta(float b) { beta = b; }
    // 角速度 gx, gy, gz の単位は rad/s
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    void retBodyAngles(float *rv);
};
#endif

