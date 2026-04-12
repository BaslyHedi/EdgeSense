/**
 * @file SensorManager.h
 * @author Hedi Basly
 * @brief Header for SensorManager module
 * @date 2026-04-06
 */
#pragma once
#include <EdgeSense/Logger/Logger.h>
#include <EdgeSense/HAL/I2cMaster.h>
#include <EdgeSense/Sensors/LPS25HB_EnvSens.h>
#include <EdgeSense/Sensors/LSM9DS1_ImuSensMag.h>
#include <EdgeSense/Sensors/LSM9DS1_ImuSensAccGyro.h>
#include <EdgeSense/Sensors/SensorsRegistry.h>
#include <EdgeSense/Core/ThreadManager.h>
#include <EdgeSense/Sensors/SensorCalib.h>

namespace EdgeSense {
    namespace Core {

    class SensorManager {
    public:
        SensorManager(ThreadManager& tm);

        /* Orchestration Logic */
        bool init(); 
        void runApplication();
        void runCalibration();
        void stop();

    private:
        /* References for external objects */
        EdgeSense::Core::ThreadManager& tm; /* Reference to the ThreadManager for task control */
        
        /* Sensor Instances */
        EdgeSense::HAL::I2cMaster I2c; /* I2C Master instance */
        std::unique_ptr <EdgeSense::Sensors::EnvSensors> Pi_LPS25HB = std::make_unique<EdgeSense::Sensors::LPS25HB>(I2c);
        std::unique_ptr <EdgeSense::Sensors::ImuSensors> Pi_LSM9DS1AG = std::make_unique<EdgeSense::Sensors::LSM9DS1_AccGyro>(I2c);
        std::unique_ptr <EdgeSense::Sensors::ImuSensors> Pi_LSM9DS1Mag = std::make_unique<EdgeSense::Sensors::LSM9DS1_Mag>(I2c);

        /* Pre-defined task builders */
        void setupAppTasks();
        void setupCalibTasks();

        /* Action Functions for APP logic */
        void appHarvestAction();
        void appRefineAction();
        void appProcessAction();
        
        /* Action Functions for CALIB logic */
        void calibHarvestAction();
        void calibProcessAction();
    };

    } /* namespace Core */
} /* namespace EdgeSense */