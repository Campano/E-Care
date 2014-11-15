/*
 * C# .NET API 0.13
 * 
 * Copyright (c) 2014, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Shimmer Research, Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Cathy Swanton, Mike Healy, Jong Chern Lim
 * @date   June, 2014
 * 
 */




using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.IO;
using System.Threading;
using System.Diagnostics;
using System.Globalization;
using System.Timers;
using UnityEngine;

namespace ShimmerAPI
{
    public class Shimmer
    {
        public const int SHIMMER_STATE_STREAMING = 3;
        public const int SHIMMER_STATE_CONNECTED = 2;
        public const int SHIMMER_STATE_CONNECTING = 1;
        public const int SHIMMER_STATE_NONE = 0;
        public const int GSR_RANGE_10K_56K = 0;
        public const int GSR_RANGE_56K_220K = 1;
        public const int GSR_RANGE_220K_680K = 2;
        public const int GSR_RANGE_680K_4700K = 3;
        public const int GSR_RANGE_AUTO = 4;
        private int ShimmerState = SHIMMER_STATE_NONE;
        public event EventHandler UICallback; //this is to be used by other classes to communicate with the C# API
        public System.IO.Ports.SerialPort SerialPort = new SerialPort();
        private System.Timers.Timer TimerConnect = null;
        private GradDes3DOrientation OrientationAlgo;
        private String DeviceName;
        private String ComPort;
        private List<ObjectCluster> ObjectClusterBuffer = new List<ObjectCluster>();
        private volatile bool StopReading = false;
        private Thread ReadThread;
        private Thread ConnectThread;
        public static int MAX_NUMBER_OF_SIGNALS = 35; //used to be 11 but now 13 because of the SR30 + 8 for 3d orientation
        public static int MAX_INQUIRY_PACKET_SIZE = 42;
        private int NumberofChannels;
        private int BufferSize;
        private double FirmwareIdentifier;
        private double FirmwareVersion;
        private int FirmwareInternal;
        private String FirmwareVersionFullName;
        private List<byte> ListofSensorChannels = new List<byte>();
        private int PacketSize = 2; // Time stamp
        private int EnabledSensors;
        private int HardwareVersion = 0;
        String[] SignalNameArray = new String[MAX_NUMBER_OF_SIGNALS];
        String[] SignalDataTypeArray = new String[MAX_NUMBER_OF_SIGNALS];
        ObjectCluster KeepObjectCluster = null; // this is to keep the packet for one byte, just incase there is a dropped packet
        public double[,] AlignmentMatrixAccel = new double[3, 3] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };
        public double[,] SensitivityMatrixAccel = new double[3, 3] { { 38, 0, 0 }, { 0, 38, 0 }, { 0, 0, 38 } };
        public double[,] OffsetVectorAccel = new double[3, 1] { { 2048 }, { 2048 }, { 2048 } };
        public double[,] AlignmentMatrixGyro = new double[3, 3] { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } };
        public double[,] SensitivityMatrixGyro = new double[3, 3] { { 2.73, 0, 0 }, { 0, 2.73, 0 }, { 0, 0, 2.73 } };
        public double[,] OffsetVectorGyro = new double[3, 1] { { 1843 }, { 1843 }, { 1843 } };
        public double[,] AlignmentMatrixMag = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } };
        public double[,] SensitivityMatrixMag = new double[3, 3] { { 580, 0, 0 }, { 0, 580, 0 }, { 0, 0, 580 } };
        public double[,] OffsetVectorMag = new double[3, 1] { { 0 }, { 0 }, { 0 } };
        public double[,] AlignmentMatrixAccel2 = new double[3, 3] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };
        public double[,] SensitivityMatrixAccel2 = new double[3, 3] { { 38, 0, 0 }, { 0, 38, 0 }, { 0, 0, 38 } };
        public double[,] OffsetVectorAccel2 = new double[3, 1] { { 2048 }, { 2048 }, { 2048 } };
        private bool LowPowerMagEnabled = false;
        private bool LowPowerAccelEnabled = false;
        private bool LowPowerGyroEnabled = false;
        private bool DefaultAccelParams = true;
        private bool DefaultWRAccelParams = true;
        private bool DefaultGyroParams = true;
        private bool DefaultMagParams = true;
        private bool DefaultECGParams = true;
        private bool DefaultEMGParams = true;
        private int ListSizeGyroOnTheFly = 100;
        private int AccelRange;
        private int GyroRange;
        private int PressureResolution;
        private int MagGain;
        private int AccelSamplingRate;
        private int Mpu9150SamplingRate;
        private int GSRRange;
        private int internalExpPower = -1;
        private int magSamplingRate;
        private int ADCRawSamplingRateValue;
        private double SamplingRate;
        public double AC1 = 408;
        public double AC2 = -72;
        public double AC3 = -14383;
        public double AC4 = 332741;
        public double AC5 = 32757;
        public double AC6 = 23153;
        public double B1 = 6190;
        public double B2 = 4;
        public double MB = -32767;
        public double MC = -8711;
        public double MD = 2868;
        public double OffsetECGRALL = 2060;
        public double GainECGRALL = 175;
        public double OffsetECGLALL = 2060;
        public double GainECGLALL = 175;
        public double OffsetEMG = 2060;
        public double GainEMG = 750;
        public double OffsetSGHigh = 60;
        public double VRef = 3;
        public double GainSGHigh = 551 * 2.8;
        public double OffsetSGLow = 1950;
        public double GainSGLow = 183.7 * 2.8;
        public double PacketReceptionRate = 100;
        private int CurrentLEDStatus = 0;
        public int LastKnownHeartRate = 0;
        private double ThresholdGyroOnTheFly = 1.2;
        private double LastReceivedTimeStamp = 0;
        private double CurrentTimeStampCycle = 0;
        private double LastReceivedCalibratedTimeStamp = -1;
        private double CalTimeStart;
        public long PacketLossCount = 0;
        private long ConfigSetupByte0; // for Shimmer2
        private bool FirstTimeCalTime = true;
        private bool IsFilled = false;
        private bool SetupDevice = false;
        private bool Orientation3DEnabled = false;
        private bool EnableGyroOnTheFlyCalibration = false;
        private int SetEnabledSensors = (int)SensorBitmapShimmer2.SENSOR_ACCEL;
        private int ChipID;
        private int CompatibilityCode = 0;
        private List<double> GyroXCalList = new List<double>();
        private List<double> GyroYCalList = new List<double>();
        private List<double> GyroZCalList = new List<double>();
        private List<double> GyroXRawList = new List<double>();
        private List<double> GyroYRawList = new List<double>();
        private List<double> GyroZRawList = new List<double>();

        public enum ShimmerVersion
        {
            SHIMMER1 = 0,
            SHIMMER2 = 1,
            SHIMMER2R = 2,
            SHIMMER3 = 3
        }

        public enum ShimmerIdentifier
        {
            MSG_IDENTIFIER_STATE_CHANGE = 0,
            MSG_IDENTIFIER_NOTIFICATION_MESSAGE = 1,
            MSG_IDENTIFIER_DATA_PACKET = 2,
            MSG_IDENTIFIER_PACKET_RECEPTION_RATE = 3
        }

        public enum SensorBitmapShimmer2
        {
            SENSOR_ACCEL = 0x80,
            SENSOR_GYRO = 0x40,
            SENSOR_MAG = 0x20,
            SENSOR_ECG = 0x10,
            SENSOR_EMG = 0x08,
            SENSOR_GSR = 0x04,
            SENSOR_EXP_BOARD_A7 = 0x02,
            SENSOR_EXP_BOARD_A0 = 0x01,
            SENSOR_STRAIN_GAUGE = 0x8000,
            SENSOR_HEART = 0x4000 // - this is for the Polar strap, but will not be supported in c# api, for use, use the old Shimmer Connect 0.12 or below
        }

        public enum SensorBitmapShimmer3
        {
            SENSOR_A_ACCEL = 0x80,
            SENSOR_MPU9150_GYRO = 0x040,
            SENSOR_LSM303DLHC_MAG = 0x20,
            SENSOR_GSR = 0x04,
            SENSOR_EXT_A7 = 0x02,
            SENSOR_EXT_A6 = 0x01,
            SENSOR_VBATT = 0x2000,
            SENSOR_D_ACCEL = 0x1000,
            SENSOR_EXT_A15 = 0x0800,
            SENSOR_INT_A1 = 0x0400,
            SENSOR_INT_A12 = 0x0200,
            SENSOR_INT_A13 = 0x0100,
            SENSOR_INT_A14 = 0x800000,
            SENSOR_BMP180_PRESSURE = 0x40000,
            SENSOR_EXG1_24BIT = 0x10,
            SENSOR_EXG2_24BIT = 0x08,
            SENSOR_EXG1_16BIT = 0x100000,
            SENSOR_EXG2_16BIT = 0x080000,
            SENSOR_BRIDGE_AMP = 0x8000
        }

        public enum PacketTypeShimmer2 : byte //Note that most packet
        {
            DATA_PACKET = 0x00,
            INQUIRY_COMMAND = 0x01,
            INQUIRY_RESPONSE = 0x02,
            GET_SAMPLING_RATE_COMMAND = 0x03,
            SAMPLING_RATE_RESPONSE = 0x04,
            SET_SAMPLING_RATE_COMMAND = 0x05,
            TOGGLE_LED_COMMAND = 0x06,
            START_STREAMING_COMMAND = 0x07,
            SET_SENSORS_COMMAND = 0x08,
            SET_ACCEL_RANGE_COMMAND = 0x09,
            ACCEL_RANGE_RESPONSE = 0x0A,
            GET_ACCEL_RANGE_COMMAND = 0x0B,
            SET_5V_REGULATOR_COMMAND = 0x0C,
            SET_POWER_MUX_COMMAND = 0x0D,
            SET_CONFIG_SETUP_BYTE0_Command = 0x0E,
            CONFIG_SETUP_BYTE0_RESPONSE = 0x0F,
            GET_CONFIG_SETUP_BYTE0_COMMAND = 0x10,
            SET_ACCEL_CALIBRATION_COMMAND = 0x11,
            ACCEL_CALIBRATION_RESPONSE = 0x12,
            GET_ACCEL_CALIBRATION_COMMAND = 0x13,
            SET_GYRO_CALIBRATION_COMMAND = 0x14,
            GYRO_CALIBRATION_RESPONSE = 0x15,
            GET_GYRO_CALIBRATION_COMMAND = 0x16,
            SET_MAG_CALIBRATION_COMMAND = 0x17,
            MAG_CALIBRATION_RESPONSE = 0x18,
            GET_MAG_CALIBRATION_COMMAND = 0x19,
            STOP_STREAMING_COMMAND = 0x20,
            SET_GSR_RANGE_COMMAND = 0x21,
            GSR_RANGE_RESPONSE = 0x22,
            GET_GSR_RANGE_COMMAND = 0x23,
            GET_SHIMMER_VERSION_COMMAND = 0x24,
            GET_SHIMMER_VERSION_RESPONSE = 0x25,
            SET_EMG_CALIBRATION_COMMAND = 0x26,
            EMG_CALIBRATION_RESPONSE = 0x27,
            GET_EMG_CALIBRATION_COMMAND = 0x28,
            SET_ECG_CALIBRATION_COMMAND = 0x29,
            ECG_CALIBRATION_RESPONSE = 0x2A,
            GET_ECG_CALIBRATION_COMMAND = 0x2B,
            GET_ALL_CALIBRATION_COMMAND = 0x2C,
            ALL_CALIBRATION_RESPONSE = 0x2D,
            GET_FW_VERSION_COMMAND = 0x2E,
            FW_VERSION_RESPONSE = 0x2F,
            SET_BLINK_LED = 0x30,
            BLINK_LED_RESPONSE = 0x31,
            GET_BLINK_LED = 0x32,
            SET_GYRO_TEMP_VREF_COMMAND = 0x33,
            SET_BUFFER_SIZE_COMMAND = 0x34,
            BUFFER_SIZE_RESPONSE = 0x35,
            GET_BUFFER_SIZE_COMMAND = 0x36,
            SET_MAG_GAIN_COMMAND = 0x37,
            MAG_GAIN_RESPONSE = 0x38,
            GET_MAG_GAIN_COMMAND = 0x39,
            SET_MAG_SAMPLING_RATE_COMMAND = 0x3A,
            MAG_SAMPLING_RATE_RESPONSE = 0x3B,
            GET_MAG_SAMPLING_RATE_COMMAND = 0x3C,
            ACK_COMMAND = 0xFF
        };

        public enum PacketTypeShimmer3 : byte
        {

            SET_LNACCEL_CALIBRATION_COMMAND = 0x11,
            LNACCEL_CALIBRATION_RESPONSE = 0x12,
            GET_LNACCEL_CALIBRATION_COMMAND = 0x13,
            SET_GYRO_CALIBRATION_COMMAND = 0x14,
            GYRO_CALIBRATION_RESPONSE = 0x15,
            GET_GYRO_CALIBRATION_COMMAND = 0x16,
            SET_MAG_CALIBRATION_COMMAND = 0x17,
            MAG_CALIBRATION_RESPONSE = 0x18,
            GET_MAG_CALIBRATION_COMMAND = 0x19,
            WR_ACCEL_CALIBRATION_RESPONSE = 0x1B,
            STOP_STREAMING_COMMAND = 0x20,
            GET_SHIMMER_VERSION_COMMAND = 0x3F,
            SHIMMER_VERSION_RESPONSE = 0x25,
            GET_FW_VERSION_COMMAND = 0x2E,
            FW_VERSION_RESPONSE = 0x2F,
            SET_LSM303DLHC_MAG_GAIN_COMMAND = 0x37,
            SET_ACCEL_SAMPLING_RATE_COMMAND = 0x40,
            ACCEL_SAMPLING_RATE_RESPONSE = 0x41,
            GET_ACCEL_SAMPLING_RATE_COMMAND = 0x42,
            MPU9150_GYRO_RANGE_RESPONSE = 0x4A,
            GET_MPU9150_GYRO_RANGE_COMMAND = 0x4B,
            SET_MPU9150_SAMPLING_RATE_COMMAND = 0x4C,
            SET_MPU9150_GYRO_RANGE_COMMAND = 0x49,
            SET_BMP180_PRES_RESOLUTION_COMMAND = 0x52,
            BMP180_PRES_RESOLUTION_RESPONSE = 0x53,
            GET_BMP180_PRES_RESOLUTION_COMMAND = 0x54,
            BMP180_CALIBRATION_COEFFICIENTS_RESPONSE = 0x58,
            GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = 0x59,
            SET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x5E,
            INTERNAL_EXP_POWER_ENABLE_RESPONSE = 0x5F,
            GET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x60,
            SET_EXG_REGS_COMMAND = 0x61,
            EXG_REGS_RESPONSE = 0x62,
            GET_EXG_REGS_COMMAND = 0x63,
            ACK_PROCESSED = 0xFF
        };

        public enum ConfigSetupByte0Bitmap : byte
        {
            Config5VReg = 0x80,
            ConfigPMux = 0x40,
        }

        public static readonly String[] LIST_OF_ACCEL_RANGE_SHIMMER2 = { "± 1.5g", "± 2g", "± 4g", "± 6g" };
        public static readonly String[] LIST_OF_MAG_RANGE_SHIMMER2 = { "± 0.8Ga", "± 1.3Ga", "± 1.9Ga", "± 2.5Ga", "± 4.0Ga", "± 4.7Ga", "± 5.6Ga", "± 8.1Ga" };
        public static readonly String[] LIST_OF_GSR_RANGE_SHIMMER2 = { "10kOhm to 56kOhm", "56kOhm to 220kOhm", "220kOhm to 680kOhm", "680kOhm to 4.7MOhm", "Auto Range" };
        public static readonly String[] LIST_OF_ACCEL_RANGE_SHIMMER3 = { "+/- 2g", "+/- 4g", "+/- 8g", "+/- 16g" };
        public static readonly String[] LIST_OF_GYRO_RANGE_SHIMMER3 = { "250dps", "500dps", "1000dps", "2000dps" };
        public static readonly String[] LIST_OF_MAG_RANGE_SHIMMER3 = { "+/- 1.3Ga", "+/- 1.9Ga", "+/- 2.5Ga", "+/- 4.0Ga", "+/- 4.7Ga", "+/- 5.6Ga", "+/- 8.1Ga" };
        public static readonly String[] LIST_OF_PRESSURE_RESOLUTION_SHIMMER3 = { "Low", "Standard", "High", "Very High" };
        public static readonly String[] LIST_OF_GSR_RANGE = { "10kOhm to 56kOhm", "56kOhm to 220kOhm", "220kOhm to 680kOhm", "680kOhm to 4.7MOhm", "Auto Range" };
        public static readonly String[] LIST_OF_EXG_GAINS_SHIMMER3 = new string[] { "1", "2", "3", "4", "6", "8", "12" };

        public static readonly double[,] SENSITIVITY_MATRIX_ACCEL_1_5G_Shimmer2 = new double[3, 3] { { 101, 0, 0 }, { 0, 101, 0 }, { 0, 0, 101 } };
        public static readonly double[,] SENSITIVITY_MATRIX_ACCEL_2G_SHIMMER2 = new double[3, 3] { { 76, 0, 0 }, { 0, 76, 0 }, { 0, 0, 76 } };
        public static readonly double[,] SENSITIVITY_MATRIX_ACCEL_4G_SHIMMER2 = new double[3, 3] { { 38, 0, 0 }, { 0, 38, 0 }, { 0, 0, 38 } };
        public static readonly double[,] SENSITIVITY_MATRIX_ACCEL_6G_SHIMMER2 = new double[3, 3] { { 25, 0, 0 }, { 0, 25, 0 }, { 0, 0, 25 } };
        public static readonly double[,] OFFSET_VECTOR_ACCEL_SHIMMER2 = new double[3, 1] { { 2048 }, { 2048 }, { 2048 } };				//Default Values for Accelerometer Calibration
        public static readonly double[,] ALIGNMENT_MATRIX_ACCEL_SHIMMER2 = new double[3, 3] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };
        
        public static readonly double[,] ALIGNMENT_MATRIX_GYRO_SHIMMER2 = new double[3, 3] { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } }; 				//Default Values for Gyroscope Calibration
        public static readonly double[,] SENSITIVITY_MATRIX_GYRO_SHIMMER2 = new double[3, 3] { { 2.73, 0, 0 }, { 0, 2.73, 0 }, { 0, 0, 2.73 } }; 		//Default Values for Gyroscope Calibration
        public static readonly double[,] OFFSET_VECTOR_GYRO_SHIMMER2 = new double[3, 1] { { 1843 }, { 1843 }, { 1843 } };						//Default Values for Gyroscope Calibration

        public static readonly double[,] ALIGNMENT_MATRIX_MAG_SHIMMER2 = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } }; 				//Default Values for Magnetometer Calibration
        public static readonly double[,] OFFSET_VECTOR_MAG_SHIMMER2 = new double[3, 1] { { 0 }, { 0 }, { 0 } };									//Default Values for Magnetometer Calibration
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_0_8GA_SHIMMER2 = new double[3, 3] { { 1370, 0, 0 }, { 0, 1370, 0 }, { 0, 0, 1370 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_1_3GA_SHIMMER2 = new double[3, 3] { { 1090, 0, 0 }, { 0, 1090, 0 }, { 0, 0, 1090 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_1_9GA_SHIMMER2 = new double[3, 3] { { 820, 0, 0 }, { 0, 820, 0 }, { 0, 0, 820 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_2_5GA_SHIMMER2 = new double[3, 3] { { 660, 0, 0 }, { 0, 660, 0 }, { 0, 0, 660 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_4_0GA_SHIMMER2 = new double[3, 3] { { 440, 0, 0 }, { 0, 440, 0 }, { 0, 0, 440 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_4_7GA_SHIMMER2 = new double[3, 3] { { 390, 0, 0 }, { 0, 390, 0 }, { 0, 0, 390 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_5_6GA_SHIMMER2 = new double[3, 3] { { 330, 0, 0 }, { 0, 330, 0 }, { 0, 0, 330 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_8_1GA_SHIMMER2 = new double[3, 3] { { 230, 0, 0 }, { 0, 230, 0 }, { 0, 0, 230 } };

        public static readonly double[,] ALIGNMENT_MATRIX_LOW_NOISE_ACCEL_SHIMMER3 = new double[3, 3] { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } }; 	//Default Values for Accelerometer Calibration
        public static readonly double[,] OFFSET_VECTOR_ACCEL_LOW_NOISE_SHIMMER3 = new double[3, 1] { { 2047 }, { 2047 }, { 2047 } };				//Default Values for Accelerometer Calibration
        public static readonly double[,] SENSITIVITY_MATRIX_LOW_NOISE_ACCEL_SHIMMER3 = new double[3, 3] { { 83, 0, 0 }, { 0, 83, 0 }, { 0, 0, 83 } };

        public static readonly double[,] SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_2G_SHIMMER3 = new double[3, 3] { { 1631, 0, 0 }, { 0, 1631, 0 }, { 0, 0, 1631 } };
        public static readonly double[,] SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_4G_SHIMMER3 = new double[3, 3] { { 815, 0, 0 }, { 0, 815, 0 }, { 0, 0, 815 } };
        public static readonly double[,] SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_8G_SHIMMER3 = new double[3, 3] { { 408, 0, 0 }, { 0, 408, 0 }, { 0, 0, 408 } };
        public static readonly double[,] SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_16G_SHIMMER3 = new double[3, 3] { { 135, 0, 0 }, { 0, 135, 0 }, { 0, 0, 135 } };
        public static readonly double[,] ALIGNMENT_MATRIX_WIDE_RANGE_ACCEL_SHIMMER3 = new double[3, 3] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } }; 	//Default Values for Accelerometer Calibration
        public static readonly double[,] OFFSET_VECTOR_ACCEL_WIDE_RANGE_SHIMMER3 = new double[3, 1] { { 0 }, { 0 }, { 0 } };				//Default Values for Accelerometer Calibration

        public static readonly double[,] ALIGNMENT_MATRIX_GYRO_SHIMMER3 = new double[3, 3] { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } }; 				//Default Values for Gyroscope Calibration
        public static readonly double[,] SENSITIVITIY_MATRIX_GYRO_250DPS_SHIMMER3 = new double[3, 3] { { 131, 0, 0 }, { 0, 131, 0 }, { 0, 0, 131 } }; 		//Default Values for Gyroscope Calibration
        public static readonly double[,] SENSITIVITIY_MATRIX_GYRO_500DPS_SHIMMER3 = new double[3, 3] { { 65.5, 0, 0 }, { 0, 65.5, 0 }, { 0, 0, 65.5 } }; 		//Default Values for Gyroscope Calibration
        public static readonly double[,] SENSITIVITIY_MATRIX_GYRO_1000DPS_SHIMMER3 = new double[3, 3] { { 32.8, 0, 0 }, { 0, 32.8, 0 }, { 0, 0, 32.8 } }; 		//Default Values for Gyroscope Calibration
        public static readonly double[,] SENSITIVITIY_MATRIX_GYRO_2000DPS_SHIMMER3 = new double[3, 3] { { 16.4, 0, 0 }, { 0, 16.4, 0 }, { 0, 0, 16.4 } }; 		//Default Values for Gyroscope Calibration
        public static readonly double[,] OFFSET_VECTOR_GYRO_SHIMMER3 = new double[3, 1] { { 0 }, { 0 }, { 0 } };						//Default Values for Gyroscope Calibration

        public static readonly double[,] ALIGNMENT_MATRIX_MAG_SHIMMER3 = new double[3, 3] { { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } }; 				//Default Values for Magnetometer Calibration
        public static readonly double[,] OFFSET_VECTOR_MAG_SHIMMER3 = new double[3, 1] { { 0 }, { 0 }, { 0 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_1_3GA_SHIMMER3 = new double[3,3]{ { 1100, 0, 0 }, { 0, 1100, 0 }, { 0, 0, 980 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_1_9GA_SHIMMER3 = new double[3,3]{ { 855, 0, 0 }, { 0, 855, 0 }, { 0, 0, 760 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_2_5GA_SHIMMER3 = new double[3, 3] { { 670, 0, 0 }, { 0, 670, 0 }, { 0, 0, 600 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_4_0GA_SHIMMER3 = new double[3, 3] { { 450, 0, 0 }, { 0, 450, 0 }, { 0, 0, 400 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_4_7GA_SHIMMER3 = new double[3, 3] { { 355, 0, 0 }, { 0, 355, 0 }, { 0, 0, 300 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_5_6GA_SHIMMER3 = new double[3, 3] { { 330, 0, 0 }, { 0, 330, 0 }, { 0, 0, 295 } };
        public static readonly double[,] SENSITIVITY_MATRIX_MAG_8_1GA_SHIMMER3 = new double[3, 3] { { 230, 0, 0 }, { 0, 230, 0 }, { 0, 0, 205 } };


        public int ReadTimeout = 1000; //ms
        public int WriteTimeout = 1000; //ms

        //EXG
        public byte[] Exg1RegArray = new byte[10];
        public byte[] Exg2RegArray = new byte[10];

        //This Constructor is for both Shimmer2 and Shimmer3 where upon connection the Settings on the Shimmer device is read and saved on the API; see bool variable SetupDevice
        public Shimmer(String devName, String bComPort)
        {
            DeviceName = devName;
            ComPort = bComPort;
            SetupDevice = false;
        }

        //Shimmer3 constructor, to set the Shimmer device according to specified settings upon connection
        public Shimmer(String devName, String bComPort, double samplingRate, int accelRange, int gsrRange, int setEnabledSensors, bool enableLowPowerAccel, bool enableLowPowerGyro, bool enableLowPowerMag, int gyroRange, int magRange,byte[] exg1configuration,byte[] exg2configuration)
        {
            DeviceName = devName;
            ComPort = bComPort;
            SamplingRate = samplingRate;
            AccelRange = accelRange;
            MagGain = magRange;
            GSRRange = gsrRange;
            GyroRange = gyroRange;
            SetEnabledSensors = setEnabledSensors;
            Exg1RegArray = exg1configuration;
            Exg2RegArray = exg2configuration;
            LowPowerAccelEnabled = enableLowPowerAccel;
            LowPowerGyroEnabled = enableLowPowerGyro;
            LowPowerMagEnabled = enableLowPowerMag;
            SetupDevice = true;
        }

        //Shimmer2 constructor, to set the Shimmer device according to specified settings upon connection
        public Shimmer(String devName, String bComPort, double samplingRate, int accelRange, int gsrRange, int setEnabledSensors, int magGain)
        {
            DeviceName = devName;
            ComPort = bComPort;
            SamplingRate = samplingRate;
            AccelRange = accelRange;
            MagGain = magGain;
            GSRRange = gsrRange;
            SetEnabledSensors = setEnabledSensors;
            SetupDevice = true;
        }

        public void StartConnectThread()
        {
            if (GetState() == SHIMMER_STATE_NONE)
            {

                ConnectThread = new Thread(new ThreadStart(Connect));
                ConnectThread.Name = "Connect Thread for Device: " + DeviceName;
                ConnectThread.Start();
                if (TimerConnect != null)
                {

                }
                else
                {
                    TimerConnect = new System.Timers.Timer(20000); // Set up the timer for connecting test
                    TimerConnect.Elapsed += new ElapsedEventHandler(ConnectTimerElapsed);
                }
                TimerConnect.Start(); // Enable it
            }
        }

        void ConnectTimerElapsed(object sender, ElapsedEventArgs e)
        {
            if (GetState() == SHIMMER_STATE_CONNECTING)
            {
                //Means something has gone wrong during the connecting state
                if (SerialPort.IsOpen)
                {
                    SerialPort.Close();
                }
                SetState(SHIMMER_STATE_NONE);
                EventHandler handler = UICallback;
                if (handler != null)
                {
                    String message = "Unable to connect to specified port";
                    CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_NOTIFICATION_MESSAGE, (object)message);
                    handler(this, newEventArgs);
                }
            }
            TimerConnect.Stop();
        }

        public void Connect()
        {
			//GameObject.Find ("Script").GetComponent<ReadData> ().printConsole("Connection to serial Port");
            if (!SerialPort.IsOpen)
            {
				//GameObject.Find ("Script").GetComponent<ReadData> ().printConsole("Serial Port Closed");
                try
                {
                    SerialPort.BaudRate = 2048;
                    SerialPort.PortName = ComPort;
                    SerialPort.ReadTimeout = this.ReadTimeout;
                    SerialPort.WriteTimeout = this.WriteTimeout;
                    SetState(SHIMMER_STATE_CONNECTING);
                    SerialPort.Open();
                    SerialPort.DiscardInBuffer();
                    SerialPort.DiscardOutBuffer();
                    StopReading = false;
                    ReadThread = new Thread(new ThreadStart(ReadData));
                    ReadThread.Name = "Read Thread for Device: " + DeviceName;
                    ReadThread.Start();
                    // give the shimmer time to make the changes before continuing (required?)
                    System.Threading.Thread.Sleep(500);
                    // Read Shimmer Profile
                    if (SerialPort.IsOpen)
                    {
                        // Set default firmware version values, if there is not response it means that this values remain, and the old firmware version has been detected
                        // The following are the three main identifiers used to identify the firmware version
                        FirmwareIdentifier = 1;
                        FirmwareVersion = 0;
                        FirmwareVersionFullName = "BoilerPlate 0.1.0";
                        FirmwareInternal = 0;

                        SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_FW_VERSION_COMMAND }, 0, 1);
                        System.Threading.Thread.Sleep(200);

                        SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_FW_VERSION_COMMAND }, 0, 1);
                        System.Threading.Thread.Sleep(200);

                        if (FirmwareVersion != 1.2) //Shimmer2r and Shimmer3 commands differ, using FWVersion to determine if its a Shimmer2r for the time being, future revisions of BTStream (Shimmer2r, should update the command to 3F)
                        {
                            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.GET_SHIMMER_VERSION_COMMAND }, 0, 1);
                        }
                        else
                        {
                            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_SHIMMER_VERSION_COMMAND }, 0, 1);
                        }
                        System.Threading.Thread.Sleep(400);
                        ReadBlinkLED();
                        if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2)
                        {
                            InitializeShimmer2();
                        }
                        else if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                        {
                            InitializeShimmer3();
                        }
                    }
                }
                catch
                {
					//GameObject.Find ("Script").GetComponent<ReadData> ().printConsole("entry");
                    TimerConnect.Stop(); // Enable it
					//GameObject.Find ("Script").GetComponent<ReadData> ().printConsole("exit");
                    StopReading = true;
                    ReadThread = null;
                    SetState(SHIMMER_STATE_NONE);
                    EventHandler handler = UICallback;
                    if (handler != null)
                    {
                        String message = "Unable to connect to specified port";
                        CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_NOTIFICATION_MESSAGE, (object)message);
                        handler(this, newEventArgs);
                    }
                }
            }
            else

            {
                TimerConnect.Stop(); // Enable it
                StopReading = true;
                ReadThread = null;
                SetState(SHIMMER_STATE_NONE);
                EventHandler handler = UICallback;
                if (handler != null)
                {
                    String message = "Unable to connect to specified port";
                    CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_NOTIFICATION_MESSAGE, (object)message);
                    handler(this, newEventArgs);
                }
            }
        }

        private void ReadData()
        {
            List<byte> buffer = new List<byte>();
            int i;
            byte[] bufferbyte;
            List<byte> dataByte;
            ObjectCluster objectCluster;
            SerialPort.DiscardInBuffer();
            KeepObjectCluster = null;

            while (!StopReading)
            {
                try
                {
                    byte b = (byte)SerialPort.ReadByte();
                    if (ShimmerState == SHIMMER_STATE_STREAMING)
                    {
                        switch (b)
                        {
                            case (byte)PacketTypeShimmer2.DATA_PACKET: //Shimmer3 has the same value
                                if (IsFilled)
                                {
                                    dataByte = new List<byte>();
                                    for (i = 0; i < PacketSize; i++)
                                    {
                                        dataByte.Add((byte)SerialPort.ReadByte());
                                    }

                                    objectCluster = BuildMsg(dataByte);

                                    if (KeepObjectCluster != null) // check if there was a previously received packet, if there is send that, as it is a packet without error (zero-zero test), each packet starts with zero
                                    {
                                        ObjectClusterBuffer.Add(KeepObjectCluster);
                                    }
                                    if (KeepObjectCluster != null)
                                    {
                                        EventHandler handler = UICallback;
                                        if (handler != null)
                                        {
                                            CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET, (object)KeepObjectCluster);
                                            handler(this, newEventArgs);
                                        }
                                    }
                                    KeepObjectCluster = objectCluster;
                                    buffer.Clear();
                                }
                                break;
                            case (byte)PacketTypeShimmer2.ACK_COMMAND:
                                System.Console.Write("ack while streaming");
                                break;
                            default:
                                // System.Console.WriteLine("missed packet");
                                // If it gets here means the previous packet is invalid so make it null so it wont be added to the buffer
                                System.Console.Write("error data read while streaming ~ dropped packets");
                                KeepObjectCluster = null;
                                break;
                        }
                    }
                    else
                    {
                        switch (b)
                        {
                            case (byte)PacketTypeShimmer2.DATA_PACKET:  //Read bytes but do nothing with them
                                if (IsFilled)
                                {
                                    dataByte = new List<byte>();
                                    for (i = 0; i < PacketSize; i++)
                                    {
                                        dataByte.Add((byte)SerialPort.ReadByte());
                                    }
                                    buffer.Clear();
                                }
                                break;
                            case (byte)PacketTypeShimmer2.INQUIRY_RESPONSE:
                                if (ShimmerState != SHIMMER_STATE_CONNECTED)
                                {
                                    SetState(SHIMMER_STATE_CONNECTED);
                                }
                                if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2 || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R)
                                {
                                    for (i = 0; i < 5; i++)
                                    {
                                        // get Sampling rate, accel range, config setup byte0, num chans and buffer size
                                        buffer.Add((byte)SerialPort.ReadByte());
                                    }
                                    for (i = 0; i < (int)buffer[3]; i++)
                                    {
                                        // read each channel type for the num channels
                                        buffer.Add((byte)SerialPort.ReadByte());
                                    }
                                    InterpretInquiryResponseShimmer2(buffer);
                                }
                                else if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                                {
                                    for (i = 0; i < 8; i++)
                                    {
                                        // get Sampling rate, accel range, config setup byte0, num chans and buffer size
                                        buffer.Add((byte)SerialPort.ReadByte());
                                    }
                                    for (i = 0; i < (int)buffer[6]; i++)
                                    {
                                        // read each channel type for the num channels
                                        buffer.Add((byte)SerialPort.ReadByte());
                                    }
                                    InterpretInquiryResponseShimmer3(buffer);
                                }
                                buffer.Clear();
                                break;
                            case (byte)PacketTypeShimmer2.SAMPLING_RATE_RESPONSE:
                                if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                                {
                                    int value = 0;
                                    value = (int)SerialPort.ReadByte();
                                    value += (((int)SerialPort.ReadByte() << 8) & 0xFF00);
                                    ADCRawSamplingRateValue = value;
                                    SamplingRate = (double)32768 / ADCRawSamplingRateValue;
                                }
                                else
                                {
                                    ADCRawSamplingRateValue = SerialPort.ReadByte();
                                    SamplingRate = (double)1024 / ADCRawSamplingRateValue;
                                }
                                break;
                            case (byte)PacketTypeShimmer2.ACCEL_RANGE_RESPONSE:
                                SetAccelRange(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer3.ACCEL_SAMPLING_RATE_RESPONSE:
                                SetAccelSamplingRate(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer3.MPU9150_GYRO_RANGE_RESPONSE:
                                SetGyroRange(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer2.MAG_GAIN_RESPONSE:
                                SetMagRange(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer2.MAG_SAMPLING_RATE_RESPONSE:
                                SetMagSamplingRate(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer2.CONFIG_SETUP_BYTE0_RESPONSE:
                                SetConfigSetupByte0(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer2.GSR_RANGE_RESPONSE:
                                SetGSRRange(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer3.INTERNAL_EXP_POWER_ENABLE_RESPONSE:
                                SetInternalExpPower(SerialPort.ReadByte());
                                break;
                            case (byte)PacketTypeShimmer2.ACK_COMMAND:

                                break;
                            case (byte)PacketTypeShimmer2.ACCEL_CALIBRATION_RESPONSE:
                                // size is 21 bytes
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();
                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.ACCEL_CALIBRATION_RESPONSE);
                                break;
                            case (byte)PacketTypeShimmer2.GYRO_CALIBRATION_RESPONSE:
                                // size is 21 bytes
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();

                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.GYRO_CALIBRATION_RESPONSE);
                                break;
                            case (byte)PacketTypeShimmer2.MAG_CALIBRATION_RESPONSE:
                                // size is 21 bytes
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();

                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.MAG_CALIBRATION_RESPONSE);
                                break;
                            case (byte)PacketTypeShimmer2.ALL_CALIBRATION_RESPONSE:
                                //Retrieve Accel
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();
                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.ACCEL_CALIBRATION_RESPONSE);

                                //Retrieve Gyro
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();

                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.GYRO_CALIBRATION_RESPONSE);

                                //Retrieve Mag
                                bufferbyte = new byte[21];
                                for (int p = 0; p < 21; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();

                                }
                                RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer2.MAG_CALIBRATION_RESPONSE);
                                if (HardwareVersion != (int)Shimmer.ShimmerVersion.SHIMMER3)
                                {
                                    //Retrieve EMG n ECG
                                    bufferbyte = new byte[12];
                                    for (int p = 0; p < 12; p++)
                                    {
                                        bufferbyte[p] = (byte)SerialPort.ReadByte();

                                    }
                                    if (bufferbyte[0] == 255 && bufferbyte[1] == 255 && bufferbyte[2] == 255 && bufferbyte[3] == 255)
                                    {
                                        DefaultEMGParams = true;
                                    }
                                    else
                                    {
                                        OffsetEMG = (double)((bufferbyte[0] & 0xFF) << 8) + (bufferbyte[1] & 0xFF);
                                        GainEMG = (double)((bufferbyte[2] & 0xFF) << 8) + (bufferbyte[3] & 0xFF);
                                        DefaultEMGParams = false;
                                    }
                                    if (bufferbyte[4] == 255 && bufferbyte[5] == 255 && bufferbyte[6] == 255 && bufferbyte[7] == 255)
                                    {
                                        DefaultECGParams = true;
                                    }
                                    else
                                    {
                                        OffsetECGLALL = (double)((bufferbyte[4] & 0xFF) << 8) + (bufferbyte[5] & 0xFF);
                                        GainECGLALL = (double)((bufferbyte[6] & 0xFF) << 8) + (bufferbyte[7] & 0xFF);
                                        OffsetECGRALL = (double)((bufferbyte[8] & 0xFF) << 8) + (bufferbyte[9] & 0xFF);
                                        GainECGRALL = (double)((bufferbyte[10] & 0xFF) << 8) + (bufferbyte[11] & 0xFF);
                                        DefaultECGParams = false;
                                    }
                                }
                                else
                                {
                                    //Retrieve Digital Accel Cal Paramters if Shimmer 3
                                    bufferbyte = new byte[21];
                                    for (int p = 0; p < 21; p++)
                                    {
                                        bufferbyte[p] = (byte)SerialPort.ReadByte();

                                    }
                                    RetrieveKinematicCalibrationParametersFromPacket(bufferbyte, (byte)PacketTypeShimmer3.WR_ACCEL_CALIBRATION_RESPONSE);
                                }

                                break;
                            case (byte)PacketTypeShimmer3.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE:
                                bufferbyte = new byte[22];
                                for (int p = 0; p < 22; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();
                                }
                                AC1 = Calculatetwoscomplement((int)((int)(bufferbyte[1] & 0xFF) + ((int)(bufferbyte[0] & 0xFF) << 8)), 16);
                                AC2 = Calculatetwoscomplement((int)((int)(bufferbyte[3] & 0xFF) + ((int)(bufferbyte[2] & 0xFF) << 8)), 16);
                                AC3 = Calculatetwoscomplement((int)((int)(bufferbyte[5] & 0xFF) + ((int)(bufferbyte[4] & 0xFF) << 8)), 16);
                                AC4 = (int)((int)(bufferbyte[7] & 0xFF) + ((int)(bufferbyte[6] & 0xFF) << 8));
                                AC5 = (int)((int)(bufferbyte[9] & 0xFF) + ((int)(bufferbyte[8] & 0xFF) << 8));
                                AC6 = (int)((int)(bufferbyte[11] & 0xFF) + ((int)(bufferbyte[10] & 0xFF) << 8));
                                B1 = Calculatetwoscomplement((int)((int)(bufferbyte[13] & 0xFF) + ((int)(bufferbyte[12] & 0xFF) << 8)), 16);
                                B2 = Calculatetwoscomplement((int)((int)(bufferbyte[15] & 0xFF) + ((int)(bufferbyte[14] & 0xFF) << 8)), 16);
                                MB = Calculatetwoscomplement((int)((int)(bufferbyte[17] & 0xFF) + ((int)(bufferbyte[16] & 0xFF) << 8)), 16);
                                MC = Calculatetwoscomplement((int)((int)(bufferbyte[19] & 0xFF) + ((int)(bufferbyte[18] & 0xFF) << 8)), 16);
                                MD = Calculatetwoscomplement((int)((int)(bufferbyte[21] & 0xFF) + ((int)(bufferbyte[20] & 0xFF) << 8)), 16);
                                break;
                            case (byte)PacketTypeShimmer2.BLINK_LED_RESPONSE:
                                bufferbyte = new byte[1];
                                bufferbyte[0] = (byte)SerialPort.ReadByte();
                                CurrentLEDStatus = bufferbyte[0];
                                break;
                            case (byte)PacketTypeShimmer2.FW_VERSION_RESPONSE:
                                // size is 21 bytes
                                bufferbyte = new byte[6];
                                for (int p = 0; p < 6; p++)
                                {
                                    bufferbyte[p] = (byte)SerialPort.ReadByte();

                                }
                                FirmwareIdentifier = ((double)((bufferbyte[1] & 0xFF) << 8) + (double)(bufferbyte[0] & 0xFF));
                                FirmwareVersion = ((double)((bufferbyte[3] & 0xFF) << 8) + (double)(bufferbyte[2] & 0xFF) + ((double)((bufferbyte[4] & 0xFF)) / 10));
                                FirmwareInternal = ((int)(bufferbyte[5] & 0xFF));
                                string temp = "";
                                if (FirmwareIdentifier == 1)
                                {
                                    temp = "BtStream " + FirmwareVersion.ToString("0.0") + "." + FirmwareInternal.ToString();
                                }
                                else if (FirmwareIdentifier == 3)
                                {
                                    temp = "LogAndStream " + FirmwareVersion.ToString("0.0") + "." + FirmwareInternal.ToString();
                                }
                                FirmwareVersionFullName = temp;
                                SetCompatibilityCode();
                                break;
                            case (byte)PacketTypeShimmer2.GET_SHIMMER_VERSION_RESPONSE:
                                bufferbyte = new byte[1];
                                bufferbyte[0] = (byte)SerialPort.ReadByte();
                                HardwareVersion = bufferbyte[0];
                                // set default calibration parameters
                                SetCompatibilityCode();
                                break;
                            case (byte)PacketTypeShimmer3.EXG_REGS_RESPONSE:
                                if (ChipID == 1)
                                {
                                    SerialPort.ReadByte();
                                    Exg1RegArray[0] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[1] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[2] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[3] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[4] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[5] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[6] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[7] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[8] = (byte)SerialPort.ReadByte();
                                    Exg1RegArray[9] = (byte)SerialPort.ReadByte();
                                }
                                else
                                {
                                    SerialPort.ReadByte();
                                    Exg2RegArray[0] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[1] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[2] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[3] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[4] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[5] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[6] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[7] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[8] = (byte)SerialPort.ReadByte();
                                    Exg2RegArray[9] = (byte)SerialPort.ReadByte();

                                }

                                break;
                            default:
                                break;
                        }
                    }
                }

                catch (System.TimeoutException)
                {

                }
                catch (System.InvalidOperationException)
                {

                }
                catch (System.IO.IOException)
                {

                }

            }

            // only stop reading when disconnecting, so disconnect serial port here too
            SerialPort.Close();

        }

        public bool IsConnected()
        {
            if ((ShimmerState == SHIMMER_STATE_CONNECTED || ShimmerState == SHIMMER_STATE_STREAMING) && SerialPort.IsOpen)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public void Disconnect()
        {
            try
            {
                if (ReadThread != null)
                {
                    //ReadThread.Abort();
                }
                if (ConnectThread != null)
                {
                    //ConnectThread.Abort();
                }
            }
            catch
            {
            }
            if (SerialPort.IsOpen == true)
            {
                if (GetState() == Shimmer.SHIMMER_STATE_STREAMING)
                {
                    StopStreaming();
                }
                SerialPort.DiscardInBuffer();
                SerialPort.DiscardOutBuffer();
                try
                {
                    SerialPort.Close();
                }
                catch
                {
                }
            }
            else
            {
                
            }
            ObjectClusterBuffer.Clear();
            StopReading = true;
            SetState(SHIMMER_STATE_NONE);
        }

        private void InitializeShimmer2()
        {
            if (SetupDevice == true)
            {
                WriteAccelRange(AccelRange);
                WriteGSRRange(GSRRange);
                WriteSamplingRate(SamplingRate);
                WriteSensors(SetEnabledSensors);
                WriteBufferSize(1);
            }

            ReadSamplingRate();
            ReadCalibrationParameters("All");
            
            //if (FirmwareVersion == 0.1)
            if (CompatibilityCode == 1)
            {
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_ACCEL_CALIBRATION_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(500);

                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_GYRO_CALIBRATION_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(500);

                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_MAG_CALIBRATION_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(500);
            }
            else
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_BUFFER_SIZE_COMMAND, (byte)1 }, 0, 2);
                System.Threading.Thread.Sleep(200);
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_SAMPLING_RATE_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(200);
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_MAG_GAIN_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(200);
                if (HardwareVersion != (int)Shimmer.ShimmerVersion.SHIMMER3)
                {
                    if (!LowPowerMagEnabled)
                    {
                        double samplingRate = (double)1024 / (double)ADCRawSamplingRateValue;

                        if (samplingRate > 50)
                        {
                            WriteMagSamplingRate(6);
                        }
                        else if (samplingRate > 20)
                        {
                            WriteMagSamplingRate(5);
                        }
                        else if (samplingRate > 10)
                        {
                            WriteMagSamplingRate(4);
                        }
                        else
                        {
                            WriteMagSamplingRate(3);
                        }
                    }
                    else
                    {
                        WriteMagSamplingRate(4);
                    }
                }
                else
                {
                    double samplingRate = (double)1024 / (double)ADCRawSamplingRateValue;
                    if (!LowPowerMagEnabled)
                    {
                        if (samplingRate > 102.4)
                        {
                            WriteMagSamplingRate(7);
                        }
                        else if (samplingRate > 51.2)
                        {
                            WriteMagSamplingRate(6);
                        }
                        else if (samplingRate > 10.24)
                        {
                            WriteMagSamplingRate(4);
                        }
                        else
                        {
                            WriteMagSamplingRate(4);
                        }
                    }
                    else
                    {
                        if (samplingRate >= 1)
                        {
                            WriteMagSamplingRate(4);
                        }
                        else
                        {
                            WriteMagSamplingRate(1);
                        }
                    }
                }
                ReadCalibrationParameters("All");
            }

            // Not strictly necessary here unless the GSR sensor is selected, but easier to get this value set correctly to begin with
            ReadGSRRange();
            // next obtain Accel Calibration Parameters
            Inquiry();
        }

        private void InitializeShimmer3()
        {
            if (SetupDevice == true)
            {
                WriteAccelRange(AccelRange);
                WriteGSRRange(GSRRange);
                WriteGyroRange(GyroRange);
                WriteMagRange(MagGain);
                WriteSamplingRate(SamplingRate);
                WriteSensors(SetEnabledSensors); //this should always be the last command
                SetLowPowerAccel(LowPowerAccelEnabled);
                SetLowPowerMag(LowPowerMagEnabled);
                SetLowPowerGyro(LowPowerGyroEnabled);
            }
            ReadAccelRange();
            ReadSamplingRate();
            ReadMagRange();
            ReadGyroRange();
            ReadAccelSamplingRate();
            ReadCalibrationParameters("All");
            ReadPressureCalibrationCoefficients();
            ReadEXGConfigurations(1);
            ReadEXGConfigurations(2);
            Inquiry();
        }


        public void InterpretInquiryResponseShimmer3(List<byte> packet)
        {
            //check if this packet is sane, and not just random
            if ((packet.Count >= 4))       // max number of channels currently allowable
            {
                ADCRawSamplingRateValue = (int)packet[0] + ((((int)packet[1]) << 8) & 0xFF00);
                SamplingRate = (double)32768 / ADCRawSamplingRateValue;
                ConfigSetupByte0 = (long)packet[2] + (((long)packet[3]) << 8) + (((long)packet[4]) << 16) + (((long)packet[5]) << 24);
                AccelRange = (int)((ConfigSetupByte0 >> 2) & 0x03);
                GyroRange = (int)((ConfigSetupByte0 >> 16) & 0x03);
                MagGain = (int)((ConfigSetupByte0 >> 21) & 0x07);
                AccelSamplingRate = (int)((ConfigSetupByte0 >> 4) & 0xF);
                Mpu9150SamplingRate = (int)((ConfigSetupByte0 >> 8) & 0xFF);
                magSamplingRate = (int)((ConfigSetupByte0 >> 18) & 0x07);
                PressureResolution = (int)((ConfigSetupByte0 >> 28) & 0x03);
                GSRRange = (int)((ConfigSetupByte0 >> 25) & 0x07);
                internalExpPower = (int)((ConfigSetupByte0 >> 24) & 0x01);
                if ((magSamplingRate == 4 && ADCRawSamplingRateValue < 3200)) //3200 us the raw ADC value and not in HZ
                {
                    LowPowerMagEnabled = true;
                }

                if ((AccelSamplingRate == 2 && ADCRawSamplingRateValue < 3200))
                {
                    LowPowerAccelEnabled = true;
                }

                if ((Mpu9150SamplingRate == 0xFF && ADCRawSamplingRateValue < 3200))
                {
                    LowPowerGyroEnabled = true;
                }

                NumberofChannels = (int)packet[6];
                BufferSize = (int)packet[7];
                ListofSensorChannels.Clear();

                for (int i = 0; i < NumberofChannels; i++)
                {
                    ListofSensorChannels.Add(packet[8 + i]);
                }
                byte[] signalIdArray = ListofSensorChannels.ToArray();
                InterpretDataPacketFormat(NumberofChannels, signalIdArray);
                IsFilled = true;
            }
        }
        public void InterpretInquiryResponseShimmer2(List<byte> packet) // this is the inquiry
        {
            
            //check if this packet is sane, and not just random
            if ((packet.Count >= 5))
            {
                ADCRawSamplingRateValue = (int)packet[0];
                SamplingRate = (double)1024 / ADCRawSamplingRateValue;
                AccelRange = (int)packet[1];
                ConfigSetupByte0 = (int)packet[2];
                NumberofChannels = (int)packet[3];
                BufferSize = (int)packet[4];

                ListofSensorChannels.Clear();
                for (int i = 0; i < NumberofChannels; i++)
                {
                    ListofSensorChannels.Add(packet[5 + i]);
                }
                byte[] signalIdArray = ListofSensorChannels.ToArray();
                InterpretDataPacketFormat(NumberofChannels, signalIdArray);
                IsFilled = true;

                
            }
        }



        private void RetrieveKinematicCalibrationParametersFromPacket(byte[] bufferCalibrationParameters, byte packetType)
        {
            String[] dataType = { "i16", "i16", "i16", "i16", "i16", "i16", "i8", "i8", "i8", "i8", "i8", "i8", "i8", "i8", "i8" };
            int[] formattedPacket = FormatDataPacketReverse(bufferCalibrationParameters, dataType); // using the datatype the calibration parameters are converted
            double[] AM = new double[9];
            for (int i = 0; i < 9; i++)
            {
                AM[i] = ((double)formattedPacket[6 + i]) / 100;
            }

            double[,] alignmentMatrix = new double[3, 3] { { AM[0], AM[1], AM[2] }, { AM[3], AM[4], AM[5] }, { AM[6], AM[7], AM[8] } };
            double[,] sensitivityMatrix = new double[3, 3] { { formattedPacket[3], 0, 0 }, { 0, formattedPacket[4], 0 }, { 0, 0, formattedPacket[5] } };
            double[,] offsetVector = { { formattedPacket[0] }, { formattedPacket[1] }, { formattedPacket[2] } };

            // the hardware version has to be checked as well 
            if (packetType == (byte)PacketTypeShimmer2.ACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2)) 
            {
                AlignmentMatrixAccel = alignmentMatrix;
                OffsetVectorAccel = offsetVector;
                SensitivityMatrixAccel = sensitivityMatrix;
                DefaultAccelParams = false;
            }
            else if (packetType == (byte)PacketTypeShimmer3.LNACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {   
                AlignmentMatrixAccel = alignmentMatrix;
                OffsetVectorAccel = offsetVector;
                SensitivityMatrixAccel = sensitivityMatrix;
                DefaultAccelParams = false;
            }
            else if (packetType == (byte)PacketTypeShimmer2.ACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2))
            {
                DefaultAccelParams = true;
                if (AccelRange == 0)
                {
                    SensitivityMatrixAccel = SENSITIVITY_MATRIX_ACCEL_1_5G_Shimmer2;
                }
                else if (AccelRange == 1)
                {
                    SensitivityMatrixAccel = SENSITIVITY_MATRIX_ACCEL_2G_SHIMMER2;
                }
                else if (AccelRange == 2)
                {
                    SensitivityMatrixAccel = SENSITIVITY_MATRIX_ACCEL_4G_SHIMMER2;
                }
                else if (AccelRange == 3)
                {
                    SensitivityMatrixAccel = SENSITIVITY_MATRIX_ACCEL_6G_SHIMMER2;
                }
                AlignmentMatrixAccel = ALIGNMENT_MATRIX_ACCEL_SHIMMER2;
                OffsetVectorAccel = OFFSET_VECTOR_ACCEL_SHIMMER2;
                
            }
            else if (packetType == (byte)PacketTypeShimmer3.LNACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {
                DefaultAccelParams = true;
                SensitivityMatrixAccel = SENSITIVITY_MATRIX_LOW_NOISE_ACCEL_SHIMMER3;
                AlignmentMatrixAccel = ALIGNMENT_MATRIX_LOW_NOISE_ACCEL_SHIMMER3;
                OffsetVectorAccel = OFFSET_VECTOR_ACCEL_LOW_NOISE_SHIMMER3;
            }
            else if (packetType == (byte)PacketTypeShimmer3.WR_ACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1)
            {   
                AlignmentMatrixAccel2 = alignmentMatrix;
                OffsetVectorAccel2 = offsetVector;
                SensitivityMatrixAccel2 = sensitivityMatrix;
                DefaultWRAccelParams = false;
            }
            else if (packetType == (byte)PacketTypeShimmer3.WR_ACCEL_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1)
            {
                DefaultWRAccelParams = true;

                if (AccelRange == 0)
                {
                    SensitivityMatrixAccel2 = SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_2G_SHIMMER3;
                }
                else if (AccelRange == 1)
                {
                    SensitivityMatrixAccel2 = SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_4G_SHIMMER3;
                }
                else if (AccelRange == 2)
                {
                    SensitivityMatrixAccel2 = SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_8G_SHIMMER3;
                }
                else if (AccelRange == 3)
                {
                    SensitivityMatrixAccel2 = SENSITIVITY_MATRIX_WIDE_RANGE_ACCEL_16G_SHIMMER3;
                }
                AlignmentMatrixAccel2 = ALIGNMENT_MATRIX_WIDE_RANGE_ACCEL_SHIMMER3;
                OffsetVectorAccel2 = OFFSET_VECTOR_ACCEL_WIDE_RANGE_SHIMMER3;
            }
            else if (packetType == (byte)PacketTypeShimmer2.GYRO_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2))
            {   
                AlignmentMatrixGyro = alignmentMatrix;
                OffsetVectorGyro = offsetVector;
                SensitivityMatrixGyro = sensitivityMatrix;
                SensitivityMatrixGyro[0, 0] = SensitivityMatrixGyro[0, 0] / 100;
                SensitivityMatrixGyro[1, 1] = SensitivityMatrixGyro[1, 1] / 100;
                SensitivityMatrixGyro[2, 2] = SensitivityMatrixGyro[2, 2] / 100;
                DefaultGyroParams = false;
            }
            else if (packetType == (byte)PacketTypeShimmer3.GYRO_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {   
                AlignmentMatrixGyro = alignmentMatrix;
                OffsetVectorGyro = offsetVector;
                SensitivityMatrixGyro = sensitivityMatrix;
                SensitivityMatrixGyro[0, 0] = SensitivityMatrixGyro[0, 0] / 100;
                SensitivityMatrixGyro[1, 1] = SensitivityMatrixGyro[1, 1] / 100;
                SensitivityMatrixGyro[2, 2] = SensitivityMatrixGyro[2, 2] / 100;
                DefaultGyroParams = false;
            }
            else if (packetType == (byte)PacketTypeShimmer2.GYRO_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2))
            {
                DefaultGyroParams = true;
                SensitivityMatrixGyro = SENSITIVITY_MATRIX_GYRO_SHIMMER2;
                AlignmentMatrixGyro = ALIGNMENT_MATRIX_GYRO_SHIMMER2;
                OffsetVectorGyro = OFFSET_VECTOR_GYRO_SHIMMER2;
            }
            else if (packetType == (byte)PacketTypeShimmer3.GYRO_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {
                DefaultGyroParams = true;
                if (GyroRange == 0)
                {
                    SensitivityMatrixGyro = SENSITIVITIY_MATRIX_GYRO_250DPS_SHIMMER3;
                }
                else if (GyroRange == 1)
                {
                    SensitivityMatrixGyro = SENSITIVITIY_MATRIX_GYRO_500DPS_SHIMMER3;
                }
                else if (GyroRange == 2)
                {
                    SensitivityMatrixGyro = SENSITIVITIY_MATRIX_GYRO_1000DPS_SHIMMER3;
                }
                else if (GyroRange == 3)
                {
                    SensitivityMatrixGyro = SENSITIVITIY_MATRIX_GYRO_2000DPS_SHIMMER3;
                }
                AlignmentMatrixGyro = ALIGNMENT_MATRIX_GYRO_SHIMMER3;
                OffsetVectorGyro = OFFSET_VECTOR_GYRO_SHIMMER3;
               
            }
            else if (packetType == (byte)PacketTypeShimmer2.MAG_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2))
            {
                DefaultMagParams = false;
                AlignmentMatrixMag = alignmentMatrix;
                OffsetVectorMag = offsetVector;
                SensitivityMatrixMag = sensitivityMatrix;
            }
            else if (packetType == (byte)PacketTypeShimmer3.MAG_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] != -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {
                DefaultMagParams = false;
                AlignmentMatrixMag = alignmentMatrix;
                OffsetVectorMag = offsetVector;
                SensitivityMatrixMag = sensitivityMatrix;
            }
            else if (packetType == (byte)PacketTypeShimmer2.MAG_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2))
            {
                DefaultMagParams = true;
                AlignmentMatrixMag = ALIGNMENT_MATRIX_MAG_SHIMMER2;
                OffsetVectorMag = OFFSET_VECTOR_MAG_SHIMMER2;

                if (GetMagRange() == 0)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_0_8GA_SHIMMER2;
                }
                else if (GetMagRange() == 1)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_1_3GA_SHIMMER2;
                }
                else if (GetMagRange() == 2)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_1_9GA_SHIMMER2;
                }
                else if (GetMagRange() == 3)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_2_5GA_SHIMMER2;
                }
                else if (GetMagRange() == 4)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_4_0GA_SHIMMER2;
                }
                else if (GetMagRange() == 5)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_4_7GA_SHIMMER2;
                }
                else if (GetMagRange() == 6)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_5_6GA_SHIMMER2;
                }
                else if (GetMagRange() == 7)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_8_1GA_SHIMMER2;
                }
               
            }
            else if (packetType == (byte)PacketTypeShimmer3.MAG_CALIBRATION_RESPONSE && sensitivityMatrix[0, 0] == -1 && HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
            {
                DefaultMagParams = true;
                AlignmentMatrixMag = ALIGNMENT_MATRIX_MAG_SHIMMER3;
                OffsetVectorMag = OFFSET_VECTOR_MAG_SHIMMER3;
                if (GetMagRange() == 1)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_1_3GA_SHIMMER3;
                }
                else if (GetMagRange() == 2)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_1_9GA_SHIMMER3;
                }
                else if (GetMagRange() == 3)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_2_5GA_SHIMMER3;
                }
                else if (GetMagRange() == 4)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_4_0GA_SHIMMER3;
                }
                else if (GetMagRange() == 5)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_4_7GA_SHIMMER3;
                }
                else if (GetMagRange() == 6)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_5_6GA_SHIMMER3;
                }
                else if (GetMagRange() == 7)
                {
                    SensitivityMatrixMag = SENSITIVITY_MATRIX_MAG_8_1GA_SHIMMER3;
                }
                
            }

        }


        private int[] FormatDataPacketReverse(byte[] data, String[] dataType)
        {
            int iData = 0;
            int[] formattedData = new int[dataType.Length];

            for (int i = 0; i < dataType.Length; i++)
                if (dataType[i] == "u8")
                {
                    formattedData[i] = (int)data[iData];
                    iData = iData + 1;
                }
                else if (dataType[i] == "i8")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)0xFF & data[iData]), 8);
                    iData = iData + 1;
                }
                else if (dataType[i] == "u12")
                {

                    formattedData[i] = (int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8));
                    iData = iData + 2;
                }
                else if (dataType[i] == "u16")
                {

                    formattedData[i] = (int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8));
                    iData = iData + 2;
                }
                else if (dataType[i] == "i16")
                {

                    formattedData[i] = Calculatetwoscomplement((int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8)), 16);
                    iData = iData + 2;
                }
            return formattedData;
        }

        private int Calculatetwoscomplement(int signedData, int bitLength)
        {
            int newData = signedData;
            if (signedData >= (1 << (bitLength - 1)))
            {
                newData = -((signedData ^ (int)(Math.Pow(2, bitLength) - 1)) + 1);
            }

            return newData;
        }



        protected void InterpretDataPacketFormat(int nC, byte[] signalid)
        {
            String[] signalNameArray = new String[MAX_NUMBER_OF_SIGNALS];
            String[] signalDataTypeArray = new String[MAX_NUMBER_OF_SIGNALS];
            signalNameArray[0] = "TimeStamp";
            signalDataTypeArray[0] = "u16";
            int packetSize = 2; // Time stamp
            int enabledSensors = 0x00;
            for (int i = 0; i < nC; i++)
            {
                if ((byte)signalid[i] == (byte)0x00)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Low Noise Accelerometer X";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_A_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Accelerometer X";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ACCEL);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x01)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Low Noise Accelerometer Y";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_A_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Accelerometer Y";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ACCEL);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x02)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Low Noise Accelerometer Z";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Accelerometer Z";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ACCEL);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x03)
                {

                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "VSenseBatt"; //should be the battery but this will do for now
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_VBATT);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Gyroscope X";
                        signalDataTypeArray[i + 1] = "u12";

                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_GYRO);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x04)
                {

                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        signalNameArray[i + 1] = "Wide Range Accelerometer X";
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_D_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Gyroscope Y";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_GYRO);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x05)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        signalNameArray[i + 1] = "Wide Range Accelerometer Y";
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_D_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Gyroscope Z";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_GYRO);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x06)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        signalNameArray[i + 1] = "Wide Range Accelerometer Z";
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_D_ACCEL);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Magnetometer X";
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_MAG);
                    }


                }
                else if ((byte)signalid[i] == (byte)0x07)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Magnetometer X";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG);
                    }
                    else
                    {
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        signalNameArray[i + 1] = "Magnetometer Y";
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_MAG);
                    }


                }
                else if ((byte)signalid[i] == (byte)0x08)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Magnetometer Y";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG);
                    }
                    else
                    {
                        signalDataTypeArray[i + 1] = "i16";
                        packetSize = packetSize + 2;
                        signalNameArray[i + 1] = "Magnetometer Z";
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_MAG);
                    }

                }
                else if ((byte)signalid[i] == (byte)0x09)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Magnetometer Z";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "ECG RA LL";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ECG);
                    }


                }
                else if ((byte)signalid[i] == (byte)0x0A)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Gyroscope X";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_MPU9150_GYRO);
                    }
                    else
                    {

                        signalNameArray[i + 1] = "ECG LA LL";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_ECG);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x0B)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Gyroscope Y";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_MPU9150_GYRO);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "GSR Raw";
                        signalDataTypeArray[i + 1] = "u16";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_GSR);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x0C)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Gyroscope Z";
                        signalDataTypeArray[i + 1] = "i16*";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_MPU9150_GYRO);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "GSR Res";
                        signalDataTypeArray[i + 1] = "u16";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_GSR);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x0D)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "External ADC A7";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXT_A7);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "EMG";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_EMG);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x0E)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "External ADC A6";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXT_A6);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Exp Board A0";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A0);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x0F)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "External ADC A15";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXT_A15);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Exp Board A7";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A7);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x10)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Internal ADC A1";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_INT_A1);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Strain Gauge High";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x11)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Internal ADC A12";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_INT_A12);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Strain Gauge Low";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x12)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Internal ADC A13";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_INT_A13);
                    }
                    else
                    {
                        signalNameArray[i + 1] = "Heart Rate";
                        //if (FirmwareVersion == 0.1)
                        if (CompatibilityCode == 1)
                        {
                            signalDataTypeArray[i + 1] = "u8";
                            packetSize = packetSize + 1;
                        }
                        else
                        {
                            signalDataTypeArray[i + 1] = "u16";
                            packetSize = packetSize + 2;
                        }
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer2.SENSOR_HEART);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x13)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Internal ADC A14";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_INT_A14);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1A)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Temperature";
                        signalDataTypeArray[i + 1] = "u16r";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_BMP180_PRESSURE);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1B)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Pressure";
                        signalDataTypeArray[i + 1] = "u24r";
                        packetSize = packetSize + 3;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_BMP180_PRESSURE);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1C)
                {
                    if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "GSR Raw";
                        signalDataTypeArray[i + 1] = "u16";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_GSR);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1D)
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG1 Sta";
                        signalDataTypeArray[i + 1] = "u8";
                        packetSize = packetSize + 1;
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1E)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG1 CH1";
                        signalDataTypeArray[i + 1] = "i24r";
                        packetSize = packetSize + 3;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x1F)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG1 CH2";
                        signalDataTypeArray[i + 1] = "i24r";
                        packetSize = packetSize + 3;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x20)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG2 Sta";
                        signalDataTypeArray[i + 1] = "u8";
                        packetSize = packetSize + 1;
                    }
                }
                else if ((byte)signalid[i] == (byte)0x21)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG2 CH1";
                        signalDataTypeArray[i + 1] = "i24r";
                        packetSize = packetSize + 3;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x22)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG2 CH2";
                        signalDataTypeArray[i + 1] = "i24r";
                        packetSize = packetSize + 3;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x23)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG1 CH1 16Bit";
                        signalDataTypeArray[i + 1] = "i16r";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x24)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG1 CH2 16Bit";
                        signalDataTypeArray[i + 1] = "i16r";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x25)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG2 CH1 16Bit";
                        signalDataTypeArray[i + 1] = "i16r";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x26)//EXG
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "EXG2 CH2 16Bit";
                        signalDataTypeArray[i + 1] = "i16r";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x27)//BRIDGE AMPLIFIER
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Bridge Amplifier High";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP);
                    }
                }
                else if ((byte)signalid[i] == (byte)0x28)//BRIDGE AMPLIFIER
                {
                    if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                    {
                        signalNameArray[i + 1] = "Bridge Amplifier Low";
                        signalDataTypeArray[i + 1] = "u12";
                        packetSize = packetSize + 2;
                        enabledSensors = (enabledSensors | (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP);
                    }
                }
                else
                {
                    signalNameArray[i + 1] = "";
                    signalDataTypeArray[i + 1] = "u12";
                    packetSize = packetSize + 2;
                }

            }
            EnabledSensors = enabledSensors;
            SignalNameArray = signalNameArray;
            SignalDataTypeArray = signalDataTypeArray;
            PacketSize = packetSize;
        }

        protected ObjectCluster BuildMsg(List<byte> packet)
        {
            ObjectCluster objectCluster = new ObjectCluster(SerialPort.PortName, GetDeviceName());
            byte[] newPacketByte = packet.ToArray();
            long[] newPacket = ParseData(newPacketByte, SignalDataTypeArray);

            int iTimeStamp = getSignalIndex("TimeStamp"); //find index
            objectCluster.Add("Timestamp", "RAW", "no units", newPacket[iTimeStamp]);
            objectCluster.Add("Timestamp", "CAL", "mSecs", CalibrateTimeStamp(newPacket[iTimeStamp]));

            double[] accelerometer = new double[3];
            double[] gyroscope = new double[3];
            double[] magnetometer = new double[3];

            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_A_ACCEL) > 0))
                {
                    int iAccelX = getSignalIndex("Low Noise Accelerometer X"); //find index
                    int iAccelY = getSignalIndex("Low Noise Accelerometer Y"); //find index
                    int iAccelZ = getSignalIndex("Low Noise Accelerometer Z"); //find index
                    double[] datatemp = new double[3] { newPacket[iAccelX], newPacket[iAccelY], newPacket[iAccelZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixAccel, SensitivityMatrixAccel, OffsetVectorAccel);
                    string units;
                    if (DefaultAccelParams)
                    {
                        units = "m/(sec^2)*";
                    }
                    else
                    {
                        units = "m/(sec^2)";
                    }
                    objectCluster.Add("Low Noise Accelerometer X", "RAW", "no units", newPacket[iAccelX]);
                    objectCluster.Add("Low Noise Accelerometer X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Low Noise Accelerometer Y", "RAW", "no units", newPacket[iAccelY]);
                    objectCluster.Add("Low Noise Accelerometer Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Low Noise Accelerometer Z", "RAW", "no units", newPacket[iAccelZ]);
                    objectCluster.Add("Low Noise Accelerometer Z", "CAL", units, datatemp[2]);
                    accelerometer[0] = datatemp[0];
                    accelerometer[1] = datatemp[1];
                    accelerometer[2] = datatemp[2];
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_D_ACCEL) > 0))
                {
                    int iAccelX = getSignalIndex("Wide Range Accelerometer X"); //find index
                    int iAccelY = getSignalIndex("Wide Range Accelerometer Y"); //find index
                    int iAccelZ = getSignalIndex("Wide Range Accelerometer Z"); //find index
                    double[] datatemp = new double[3] { newPacket[iAccelX], newPacket[iAccelY], newPacket[iAccelZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixAccel2, SensitivityMatrixAccel2, OffsetVectorAccel2);
                    string units;
                    if (DefaultWRAccelParams)
                    {
                        units = "m/(sec^2)*";
                    }
                    else
                    {
                        units = "m/(sec^2)";
                    }
                    objectCluster.Add("Wide Range Accelerometer X", "RAW", "no units", newPacket[iAccelX]);
                    objectCluster.Add("Wide Range Accelerometer X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Wide Range Accelerometer Y", "RAW", "no units", newPacket[iAccelY]);
                    objectCluster.Add("Wide Range Accelerometer Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Wide Range Accelerometer Z", "RAW", "no units", newPacket[iAccelZ]);
                    objectCluster.Add("Wide Range Accelerometer Z", "CAL", units, datatemp[2]);

                    accelerometer[0] = datatemp[0];
                    accelerometer[1] = datatemp[1];
                    accelerometer[2] = datatemp[2];

                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_MPU9150_GYRO) > 0))
                {
                    int iGyroX = getSignalIndex("Gyroscope X");
                    int iGyroY = getSignalIndex("Gyroscope Y");
                    int iGyroZ = getSignalIndex("Gyroscope Z");
                    double[] datatemp = new double[3] { newPacket[iGyroX], newPacket[iGyroY], newPacket[iGyroZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixGyro, SensitivityMatrixGyro, OffsetVectorGyro);
                    string units;
                    if (DefaultGyroParams)
                    {
                        units = "deg/sec*";
                    }
                    else
                    {
                        units = "deg/sec";
                    }
                    objectCluster.Add("Gyroscope X", "RAW", "no units", newPacket[iGyroX]);
                    objectCluster.Add("Gyroscope X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Gyroscope Y", "RAW", "no units", newPacket[iGyroY]);
                    objectCluster.Add("Gyroscope Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Gyroscope Z", "RAW", "no units", newPacket[iGyroZ]);
                    objectCluster.Add("Gyroscope Z", "CAL", units, datatemp[2]);

                    gyroscope[0] = datatemp[0] * Math.PI / 180;
                    gyroscope[1] = datatemp[1] * Math.PI / 180;
                    gyroscope[2] = datatemp[2] * Math.PI / 180;

                    if (EnableGyroOnTheFlyCalibration)
                    {
                        GyroXRawList.Add(newPacket[iGyroX]);
                        GyroYRawList.Add(newPacket[iGyroY]);
                        GyroZRawList.Add(newPacket[iGyroZ]);
                        if (GyroXRawList.Count > ListSizeGyroOnTheFly)
                        {
                            GyroXRawList.RemoveAt(0);
                            GyroYRawList.RemoveAt(0);
                            GyroZRawList.RemoveAt(0);
                        }
                        GyroXCalList.Add(datatemp[0]);
                        GyroYCalList.Add(datatemp[1]);
                        GyroZCalList.Add(datatemp[2]);
                        if (GyroXCalList.Count > ListSizeGyroOnTheFly)
                        {
                            GyroXCalList.RemoveAt(0);
                            GyroYCalList.RemoveAt(0);
                            GyroZCalList.RemoveAt(0);

                            if (GetStandardDeviation(GyroXCalList) < ThresholdGyroOnTheFly && GetStandardDeviation(GyroYCalList) < ThresholdGyroOnTheFly && GetStandardDeviation(GyroZCalList) < ThresholdGyroOnTheFly)
                            {
                                OffsetVectorGyro[0, 0] = GyroXRawList.Average();
                                OffsetVectorGyro[1, 0] = GyroYRawList.Average();
                                OffsetVectorGyro[2, 0] = GyroZRawList.Average();
                            }
                        }
                    }
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG) > 0))
                {
                    int iMagX = getSignalIndex("Magnetometer X");
                    int iMagY = getSignalIndex("Magnetometer Y");
                    int iMagZ = getSignalIndex("Magnetometer Z");
                    double[] datatemp = new double[3] { newPacket[iMagX], newPacket[iMagY], newPacket[iMagZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixMag, SensitivityMatrixMag, OffsetVectorMag);
                    string units;
                    if (DefaultMagParams)
                    {
                        units = "local*";
                    }
                    else
                    {
                        units = "local";
                    }
                    objectCluster.Add("Magnetometer X", "RAW", "no units", newPacket[iMagX]);
                    objectCluster.Add("Magnetometer X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Magnetometer Y", "RAW", "no units", newPacket[iMagY]);
                    objectCluster.Add("Magnetometer Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Magnetometer Z", "RAW", "no units", newPacket[iMagZ]);
                    objectCluster.Add("Magnetometer Z", "CAL", units, datatemp[2]);

                    magnetometer[0] = datatemp[0];
                    magnetometer[1] = datatemp[1];
                    magnetometer[2] = datatemp[2];
                }

                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_VBATT) > 0))
                {
                    int index = getSignalIndex("VSenseBatt");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1) * 1.988);
                    if (datatemp < 3400 && datatemp > 3000)
                    {
                        //System.Threading.Thread.Sleep(500);
                        if (CurrentLEDStatus != 1)
                        {
                            SerialPort.Write(new byte[2] { (byte)Shimmer.PacketTypeShimmer2.SET_BLINK_LED, (byte)1 }, 0, 2);
                            CurrentLEDStatus = 1;
                        }
                    }
                    else if (datatemp <= 3000)
                    {
                        //System.Threading.Thread.Sleep(500);
                        if (CurrentLEDStatus != 2)
                        {
                            SerialPort.Write(new byte[2] { (byte)Shimmer.PacketTypeShimmer2.SET_BLINK_LED, (byte)2 }, 0, 2);
                            CurrentLEDStatus = 2;
                        }
                    }
                    else
                    {
                        if (CurrentLEDStatus != 0)
                        {
                            SerialPort.Write(new byte[2] { (byte)Shimmer.PacketTypeShimmer2.SET_BLINK_LED, (byte)0 }, 0, 2);
                            CurrentLEDStatus = 0;
                        }
                    }
                    objectCluster.Add("VSenseBatt", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("VSenseBatt", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXT_A7) > 0))
                {
                    int index = getSignalIndex("External ADC A7");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("External ADC A7", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("External ADC A7", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXT_A6) > 0))
                {
                    int index = getSignalIndex("External ADC A6");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("External ADC A6", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("External ADC A6", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXT_A15) > 0))
                {
                    int index = getSignalIndex("External ADC A15");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("External ADC A15", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("External ADC A15", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0))
                {
                    int index = getSignalIndex("Internal ADC A1");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("Internal ADC A1", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("Internal ADC A1", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0))
                {
                    int index = getSignalIndex("Internal ADC A12");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("Internal ADC A12", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("Internal ADC A12", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0))
                {
                    int index = getSignalIndex("Internal ADC A13");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("Internal ADC A13", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("Internal ADC A13", "CAL", "mVolts*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0))
                {
                    int index = getSignalIndex("Internal ADC A14");
                    double datatemp = newPacket[index];
                    datatemp = (CalibrateU12AdcValue(datatemp, 0, 3, 1));
                    objectCluster.Add("Internal ADC A14", "RAW", "no units", newPacket[index]);
                    objectCluster.Add("Internal ADC A14", "CAL", "mVolts*", datatemp);
                }

                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_BMP180_PRESSURE) > 0))
                {
                    int iUP = getSignalIndex("Pressure");
                    int iUT = getSignalIndex("Temperature");
                    double UT = (double)newPacket[iUT];
                    double UP = (double)newPacket[iUP];
                    UP = UP / Math.Pow(2, 8 - PressureResolution);
                    double[] datatemp = new double[2] { newPacket[iUP], newPacket[iUT] };
                    double[] bmp180caldata = CalibratePressureSensorData(UP, datatemp[1]);


                    objectCluster.Add("Pressure", "RAW", "no units", UP);
                    objectCluster.Add("Pressure", "CAL", "kPa*", bmp180caldata[0] / 1000);
                    objectCluster.Add("Temperature", "RAW", "no units", newPacket[iUT]);
                    objectCluster.Add("Temperature", "CAL", "Celcius*", bmp180caldata[1]);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0))
                {
                    int iGSR = getSignalIndex("GSR Raw");
                    int newGSRRange = -1; // initialized to -1 so it will only come into play if mGSRRange = 4  
                    double datatemp = newPacket[iGSR];
                    double p1 = 0, p2 = 0;
                    if (GSRRange == 4)
                    {
                        newGSRRange = (49152 & (int)datatemp) >> 14;
                    }
                    if (GSRRange == 0 || newGSRRange == 0)
                    { //Note that from FW 1.0 onwards the MSB of the GSR data contains the range
                        // the polynomial function used for calibration has been deprecated, it is replaced with a linear function
                        p1 = 0.0363;
                        p2 = -24.8617;
                    }
                    else if (GSRRange == 1 || newGSRRange == 1)
                    {
                        p1 = 0.0051;
                        p2 = -3.8357;
                    }
                    else if (GSRRange == 2 || newGSRRange == 2)
                    {
                        p1 = 0.0015;
                        p2 = -1.0067;
                    }
                    else if (GSRRange == 3 || newGSRRange == 3)
                    {
                        p1 = 4.4513e-04;
                        p2 = -0.3193;
                    }
                    datatemp = CalibrateGsrData(datatemp, p1, p2);
                    objectCluster.Add("GSR", "RAW", "no units", newPacket[iGSR]);
                    objectCluster.Add("GSR", "CAL", "kOhms*", datatemp);
                }
                if ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                {
                    int iStatus = getSignalIndex("EXG1 Sta");
                    int iCh1 = getSignalIndex("EXG1 CH1");
                    int iCh2 = getSignalIndex("EXG1 CH2");
                    double[] datatemp = new double[3] { newPacket[iStatus], newPacket[iCh1], newPacket[iCh2] };
                    int gain = ConvertEXGGainSettingToValue((Exg1RegArray[3] >> 4) & 7);
                    datatemp[1] = datatemp[1] * (((2.42 * 1000) / gain) / (Math.Pow(2, 23) - 1));
                    gain = ConvertEXGGainSettingToValue((Exg1RegArray[4] >> 4) & 7);
                    datatemp[2] = datatemp[2] * (((2.42 * 1000) / gain) / (Math.Pow(2, 23) - 1));
                    objectCluster.Add("EXG1 Sta", "RAW", "no units", newPacket[iStatus]);
                    if (IsDefaultECGConfigurationEnabled())
                    {
                        objectCluster.Add("ECG LL-RA", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("ECG LL-RA", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("ECG LA-RA", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("ECG LA-RA", "CAL", "mVolts*", datatemp[2]);
                    }
                    else if (IsDefaultEMGConfigurationEnabled())
                    {
                        objectCluster.Add("EMG CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EMG CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EMG CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EMG CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                    else
                    {
                        objectCluster.Add("EXG1 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG1 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG1 CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG1 CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                }
                if ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                {
                    int iStatus = getSignalIndex("EXG2 Sta");
                    int iCh1 = getSignalIndex("EXG2 CH1");
                    int iCh2 = getSignalIndex("EXG2 CH2");
                    double[] datatemp = new double[3] { newPacket[iStatus], newPacket[iCh1], newPacket[iCh2] };
                    int gain = ConvertEXGGainSettingToValue((Exg2RegArray[3] >> 4) & 7);
                    datatemp[1] = datatemp[1] * (((2.42 * 1000) / gain) / (Math.Pow(2, 23) - 1));
                    gain = ConvertEXGGainSettingToValue((Exg2RegArray[4] >> 4) & 7);
                    datatemp[2] = datatemp[2] * (((2.42 * 1000) / gain) / (Math.Pow(2, 23) - 1));
                    objectCluster.Add("EXG2 Sta", "RAW", "no units", newPacket[iStatus]);
                    if (IsDefaultECGConfigurationEnabled())
                    {
                        objectCluster.Add("EXG2 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("ECG Vx-RL", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("ECG Vx-RL", "CAL", "mVolts*", datatemp[2]);
                    }
                    else if (IsDefaultEMGConfigurationEnabled())
                    {
                        objectCluster.Add("EXG2 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG2 CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG2 CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                    else
                    {
                        objectCluster.Add("EXG2 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG2 CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG2 CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                }
                if ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                {
                    int iStatus = getSignalIndex("EXG1 Sta");
                    int iCh1 = getSignalIndex("EXG1 CH1 16Bit");
                    int iCh2 = getSignalIndex("EXG1 CH2 16Bit");
                    double[] datatemp = new double[3] { newPacket[iStatus], newPacket[iCh1], newPacket[iCh2] };
                    int gain = ConvertEXGGainSettingToValue((Exg1RegArray[3] >> 4) & 7);
                    datatemp[1] = datatemp[1] * (((2.42 * 1000) / (gain * 2)) / (Math.Pow(2, 15) - 1));
                    gain = ConvertEXGGainSettingToValue((Exg1RegArray[4] >> 4) & 7);
                    datatemp[2] = datatemp[2] * (((2.42 * 1000) / (gain * 2)) / (Math.Pow(2, 15) - 1));
                    objectCluster.Add("EXG1 Sta", "RAW", "no units", newPacket[iStatus]);
                    if (IsDefaultECGConfigurationEnabled())
                    {
                        objectCluster.Add("ECG LL-RA", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("ECG LL-RA", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("ECG LA-RA", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("ECG LA-RA", "CAL", "mVolts*", datatemp[2]);
                    }
                    else if (IsDefaultEMGConfigurationEnabled())
                    {
                        objectCluster.Add("EMG CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EMG CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EMG CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EMG CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                    else
                    {
                        objectCluster.Add("EXG1 CH1 16Bit", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG1 CH1 16Bit", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG1 CH2 16Bit", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG1 CH2 16Bit", "CAL", "mVolts*", datatemp[2]);
                    }
                }
                if ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                {
                    int iStatus = getSignalIndex("EXG2 Sta");
                    int iCh1 = getSignalIndex("EXG2 CH1 16Bit");
                    int iCh2 = getSignalIndex("EXG2 CH2 16Bit");
                    double[] datatemp = new double[3] { newPacket[iStatus], newPacket[iCh1], newPacket[iCh2] };
                    int gain = ConvertEXGGainSettingToValue((Exg2RegArray[3] >> 4) & 7);
                    datatemp[1] = datatemp[1] * (((2.42 * 1000) / (gain * 2)) / (Math.Pow(2, 15) - 1));
                    gain = ConvertEXGGainSettingToValue((Exg2RegArray[4] >> 4) & 7);
                    datatemp[2] = datatemp[2] * (((2.42 * 1000) / (gain * 2)) / (Math.Pow(2, 15) - 1));
                    objectCluster.Add("EXG2 Sta", "RAW", "no units", newPacket[iStatus]);
                    if (IsDefaultECGConfigurationEnabled())
                    {
                        objectCluster.Add("EXG2 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("ECG Vx-RL", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("ECG Vx-RL", "CAL", "mVolts*", datatemp[2]);
                    }
                    else if (IsDefaultEMGConfigurationEnabled())
                    {
                        objectCluster.Add("EXG2 CH1", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG2 CH2", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG2 CH2", "CAL", "mVolts*", datatemp[2]);
                    }
                    else
                    {
                        objectCluster.Add("EXG2 CH1 16Bit", "RAW", "no units", newPacket[iCh1]);
                        objectCluster.Add("EXG2 CH1 16Bit", "CAL", "mVolts*", datatemp[1]);
                        objectCluster.Add("EXG2 CH2 16Bit", "RAW", "no units", newPacket[iCh2]);
                        objectCluster.Add("EXG2 CH2 16Bit", "CAL", "mVolts*", datatemp[2]);
                    }
                }
                if ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                {
                    int iSGHigh = getSignalIndex("Bridge Amplifier High");
                    int iSGLow = getSignalIndex("Bridge Amplifier Low");
                    double[] datatemp = new double[2] { newPacket[iSGHigh], newPacket[iSGLow] };
                    datatemp[0] = CalibrateU12AdcValue(datatemp[0], OffsetSGHigh, VRef, GainSGHigh);
                    datatemp[1] = CalibrateU12AdcValue(datatemp[1], OffsetSGLow, VRef, GainSGLow);
                    objectCluster.Add("Bridge Amplifier High", "RAW", "no units", newPacket[iSGHigh]);
                    objectCluster.Add("Bridge Amplifier High", "CAL", "mVolts*", datatemp[0]);
                    objectCluster.Add("Bridge Amplifier Low", "RAW", "no units", newPacket[iSGLow]);
                    objectCluster.Add("Bridge Amplifier Low", "CAL", "mVolts*", datatemp[1]);
                }
                if ((((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_A_ACCEL) > 0) || ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_D_ACCEL) > 0)) 
                    && ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_MPU9150_GYRO) > 0) && ((EnabledSensors & (int)SensorBitmapShimmer3.SENSOR_LSM303DLHC_MAG) > 0) 
                    && Orientation3DEnabled)
                {
                    if (OrientationAlgo == null)
                    {
                        OrientationAlgo = new GradDes3DOrientation(0.4, (1 / this.GetSamplingRate()), 1, 0, 0, 0);
                    }
                    Quaternion q = OrientationAlgo.update(accelerometer[0], accelerometer[1], accelerometer[2], gyroscope[0], gyroscope[1], gyroscope[2], magnetometer[0], magnetometer[1], magnetometer[2]);
                    double theta, Rx, Ry, Rz, rho;
                    rho = Math.Acos(q.q1);
                    theta = rho * 2;
                    Rx = q.q2 / Math.Sin(rho);
                    Ry = q.q3 / Math.Sin(rho);
                    Rz = q.q4 / Math.Sin(rho);
                    objectCluster.Add("Axis Angle A", "CAL", "local", theta);
                    objectCluster.Add("Axis Angle X", "CAL", "local", Rx);
                    objectCluster.Add("Axis Angle Y", "CAL", "local", Ry);
                    objectCluster.Add("Axis Angle Z", "CAL", "local", Rz);
                    objectCluster.Add("Quaternion 0", "CAL", "local", q.q1);
                    objectCluster.Add("Quaternion 1", "CAL", "local", q.q2);
                    objectCluster.Add("Quaternion 2", "CAL", "local", q.q3);
                    objectCluster.Add("Quaternion 3", "CAL", "local", q.q4);
                }
            }
            else
            { //start of Shimmer2

                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_ACCEL) > 0))
                {
                    int iAccelX = getSignalIndex("Accelerometer X"); //find index
                    int iAccelY = getSignalIndex("Accelerometer Y"); //find index
                    int iAccelZ = getSignalIndex("Accelerometer Z"); //find index
                    double[] datatemp = new double[3] { newPacket[iAccelX], newPacket[iAccelY], newPacket[iAccelZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixAccel, SensitivityMatrixAccel, OffsetVectorAccel);
                    string units;
                    if (DefaultAccelParams)
                    {
                        units = "m/(sec^2)*";
                    }
                    else
                    {
                        units = "m/(sec^2)";
                    }
                    objectCluster.Add("Accelerometer X", "RAW", "no units", newPacket[iAccelX]);
                    objectCluster.Add("Accelerometer X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Accelerometer Y", "RAW", "no units", newPacket[iAccelY]);
                    objectCluster.Add("Accelerometer Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Accelerometer Z", "RAW", "no units", newPacket[iAccelZ]);
                    objectCluster.Add("Accelerometer Z", "CAL", units, datatemp[2]);
                    accelerometer[0] = datatemp[0];
                    accelerometer[1] = datatemp[1];
                    accelerometer[2] = datatemp[2];
                }

                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0))
                {
                    int iGyroX = getSignalIndex("Gyroscope X");
                    int iGyroY = getSignalIndex("Gyroscope Y");
                    int iGyroZ = getSignalIndex("Gyroscope Z");
                    double[] datatemp = new double[3] { newPacket[iGyroX], newPacket[iGyroY], newPacket[iGyroZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixGyro, SensitivityMatrixGyro, OffsetVectorGyro);
                    string units;
                    if (DefaultGyroParams)
                    {
                        units = "deg/sec*";
                    }
                    else
                    {
                        units = "deg/sec";
                    }
                    objectCluster.Add("Gyroscope X", "RAW", "no units", newPacket[iGyroX]);
                    objectCluster.Add("Gyroscope X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Gyroscope Y", "RAW", "no units", newPacket[iGyroY]);
                    objectCluster.Add("Gyroscope Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Gyroscope Z", "RAW", "no units", newPacket[iGyroZ]);
                    objectCluster.Add("Gyroscope Z", "CAL", units, datatemp[2]);

                    gyroscope[0] = datatemp[0] * Math.PI / 180;
                    gyroscope[1] = datatemp[1] * Math.PI / 180;
                    gyroscope[2] = datatemp[2] * Math.PI / 180;
 
                    if (EnableGyroOnTheFlyCalibration)
                    {
                        GyroXRawList.Add(newPacket[iGyroX]);
                        GyroYRawList.Add(newPacket[iGyroY]);
                        GyroZRawList.Add(newPacket[iGyroZ]);
                        if (GyroXRawList.Count > ListSizeGyroOnTheFly)
                        {
                            GyroXRawList.RemoveAt(0);
                            GyroYRawList.RemoveAt(0);
                            GyroZRawList.RemoveAt(0);
                        }
                        GyroXCalList.Add(datatemp[0]);
                        GyroYCalList.Add(datatemp[1]);
                        GyroZCalList.Add(datatemp[2]);
                        if (GyroXCalList.Count > ListSizeGyroOnTheFly)
                        {
                            GyroXCalList.RemoveAt(0);
                            GyroYCalList.RemoveAt(0);
                            GyroZCalList.RemoveAt(0);

                            if (GetStandardDeviation(GyroXCalList) < ThresholdGyroOnTheFly && GetStandardDeviation(GyroYCalList) < ThresholdGyroOnTheFly && GetStandardDeviation(GyroZCalList) < ThresholdGyroOnTheFly)
                            {
                                OffsetVectorGyro[0, 0] = GyroXRawList.Average();
                                OffsetVectorGyro[1, 0] = GyroYRawList.Average();
                                OffsetVectorGyro[2, 0] = GyroZRawList.Average();
                            }
                        }
                    }
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0))
                {
                    int iMagX = getSignalIndex("Magnetometer X");
                    int iMagY = getSignalIndex("Magnetometer Y");
                    int iMagZ = getSignalIndex("Magnetometer Z");
                    double[] datatemp = new double[3] { newPacket[iMagX], newPacket[iMagY], newPacket[iMagZ] };
                    datatemp = CalibrateInertialSensorData(datatemp, AlignmentMatrixMag, SensitivityMatrixMag, OffsetVectorMag);
                    string units;
                    if (DefaultMagParams)
                    {
                        units = "local*";
                    }
                    else
                    {
                        units = "local";
                    }
                    objectCluster.Add("Magnetometer X", "RAW", "no units", newPacket[iMagX]);
                    objectCluster.Add("Magnetometer X", "CAL", units, datatemp[0]);
                    objectCluster.Add("Magnetometer Y", "RAW", "no units", newPacket[iMagY]);
                    objectCluster.Add("Magnetometer Y", "CAL", units, datatemp[1]);
                    objectCluster.Add("Magnetometer Z", "RAW", "no units", newPacket[iMagZ]);
                    objectCluster.Add("Magnetometer Z", "CAL", units, datatemp[2]);

                    magnetometer[0] = datatemp[0];
                    magnetometer[1] = datatemp[1];
                    magnetometer[2] = datatemp[2];
                }

                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0))
                {
                    int iGSR = getSignalIndex("GSR Raw");
                    int newGSRRange = -1; // initialized to -1 so it will only come into play if mGSRRange = 4  
                    double datatemp = newPacket[iGSR];
                    double p1 = 0, p2 = 0;
                    if (GSRRange == 4)
                    {
                        newGSRRange = (49152 & (int)datatemp) >> 14;
                    }
                    if (GSRRange == 0 || newGSRRange == 0)
                    { //Note that from FW 1.0 onwards the MSB of the GSR data contains the range
                        // the polynomial function used for calibration has been deprecated, it is replaced with a linear function
                        p1 = 0.0373;
                        p2 = -24.9915;
                    }
                    else if (GSRRange == 1 || newGSRRange == 1)
                    {
                        p1 = 0.0054;
                        p2 = -3.5194;
                    }
                    else if (GSRRange == 2 || newGSRRange == 2)
                    {
                        p1 = 0.0015;
                        p2 = -1.0163;
                    }
                    else if (GSRRange == 3 || newGSRRange == 3)
                    {
                        p1 = 4.5580e-04;
                        p2 = -0.3014;
                    }
                    datatemp = CalibrateGsrData(datatemp, p1, p2);
                    objectCluster.Add("GSR", "RAW", "no units", newPacket[iGSR]);
                    objectCluster.Add("GSR", "CAL", "kOhms*", datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0))
                {
                    int iECGRALL = getSignalIndex("ECG RA LL");
                    int iECGLALL = getSignalIndex("ECG LA LL");
                    double[] datatemp = new double[2] { newPacket[iECGRALL], newPacket[iECGLALL] };
                    datatemp[0] = CalibrateU12AdcValue(datatemp[0], OffsetECGRALL, 3, GainECGRALL);
                    datatemp[1] = CalibrateU12AdcValue(datatemp[1], OffsetECGLALL, 3, GainECGLALL);
                    string units = "mVolts";
                    if (DefaultECGParams)
                    {
                        units = "mVolts*";
                    }
                    objectCluster.Add("ECG RA LL", "RAW", "no units", newPacket[iECGRALL]);
                    objectCluster.Add("ECG RA LL", "CAL", units, datatemp[0]);
                    objectCluster.Add("ECG LA LL", "RAW", "no units", newPacket[iECGLALL]);
                    objectCluster.Add("ECG LA LL", "CAL", units, datatemp[1]);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0))
                {
                    int iEMG = getSignalIndex("EMG");
                    double datatemp = newPacket[iEMG];
                    datatemp = CalibrateU12AdcValue(datatemp, OffsetEMG, 3, GainEMG);
                    string units = "mVolts";
                    if (DefaultEMGParams)
                    {
                        units = "mVolts*";
                    }
                    objectCluster.Add("EMG", "RAW", "no units", newPacket[iEMG]);
                    objectCluster.Add("EMG", "CAL", units, datatemp);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0))
                {
                    int iSGHigh = getSignalIndex("Strain Gauge High");
                    int iSGLow = getSignalIndex("Strain Gauge Low");
                    double[] datatemp = new double[2] { newPacket[iSGHigh], newPacket[iSGLow] };
                    datatemp[0] = CalibrateU12AdcValue(datatemp[0], OffsetSGHigh, VRef, GainSGHigh);
                    datatemp[1] = CalibrateU12AdcValue(datatemp[1], OffsetSGLow, VRef, GainSGLow);
                    objectCluster.Add("Strain Gauge High", "RAW", "no units", newPacket[iSGHigh]);
                    objectCluster.Add("Strain Gauge High", "CAL", "mVolts*", datatemp[0]);
                    objectCluster.Add("Strain Gauge Low", "RAW", "no units", newPacket[iSGLow]);
                    objectCluster.Add("Strain Gauge Low", "CAL", "mVolts*", datatemp[1]);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_HEART) > 0))
                {
                    int iHeartRate = getSignalIndex("Heart Rate");
                    double datatemp = newPacket[iHeartRate];
                    double cal = datatemp;
                    //if (FirmwareVersion == 0.1)
                    if (CompatibilityCode == 1)
                    {

                    }
                    else
                    {
                        if (datatemp == 0)
                        {
                            cal = LastKnownHeartRate;
                        }
                        else
                        {
                            cal = (int)(1024 / datatemp * 60);
                            LastKnownHeartRate = (int)cal;
                        }
                    }
                    objectCluster.Add("Heart Rate", "RAW", "no units", newPacket[iHeartRate]);
                    objectCluster.Add("Heart Rate", "CAL", "mVolts*", cal);
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A0) > 0))
                {
                    int iA0 = getSignalIndex("Exp Board A0");
                    double datatemp = newPacket[iA0];
                    datatemp = CalibrateU12AdcValue(datatemp, 0, 3, 1) * 1.988;
                    if (GetPMux())
                    {
                        objectCluster.Add("VSenseReg", "RAW", "no units", newPacket[iA0]);
                        objectCluster.Add("VSenseReg", "CAL", "mVolts*", datatemp);
                    }
                    else
                    {
                        objectCluster.Add("Exp Board A0", "RAW", "no units", newPacket[iA0]);
                        objectCluster.Add("Exp Board A0", "CAL", "mVolts*", datatemp);
                    }
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A7) > 0))
                {

                    int iA7 = getSignalIndex("Exp Board A7");
                    double datatemp = newPacket[iA7];
                    datatemp = CalibrateU12AdcValue(datatemp, 0, 3, 1) * 2;
                    if (GetPMux())
                    {
                        objectCluster.Add("VSenseBatt", "RAW", "no units", newPacket[iA7]);
                        objectCluster.Add("VSenseBatt", "CAL", "mVolts*", datatemp);
                        if (datatemp < 3400)
                        {
                            //System.Threading.Thread.Sleep(500);
                            if (CurrentLEDStatus == 0)
                            {
                                SerialPort.Write(new byte[2] { (byte)Shimmer.PacketTypeShimmer2.SET_BLINK_LED, (byte)1 }, 0, 2);
                                CurrentLEDStatus = 1;

                            }
                        }
                        else
                        {
                            //System.Threading.Thread.Sleep(500);
                            if (CurrentLEDStatus == 1)
                            {
                                SerialPort.Write(new byte[2] { (byte)Shimmer.PacketTypeShimmer2.SET_BLINK_LED, (byte)0 }, 0, 2);
                                CurrentLEDStatus = 0;

                            }
                        }
                    }
                    else
                    {
                        objectCluster.Add("Exp Board A7", "RAW", "no units", newPacket[iA7]);
                        objectCluster.Add("Exp Board A7", "CAL", "mVolts*", datatemp);
                    }
                }
                if (((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_ACCEL) > 0) && ((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0) && ((EnabledSensors & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0) && Orientation3DEnabled)
                {
                    if (OrientationAlgo == null)
                    {
                        OrientationAlgo = new GradDes3DOrientation(0.4, 1 / this.GetSamplingRate(), 1, 0, 0, 0);
                    }
                    Quaternion q = OrientationAlgo.update(accelerometer[0], accelerometer[1], accelerometer[2], gyroscope[0], gyroscope[1], gyroscope[2], magnetometer[0], magnetometer[1], magnetometer[2]);
                    double theta, Rx, Ry, Rz, rho;
                    rho = Math.Acos(q.q1);
                    theta = rho * 2;
                    Rx = q.q2 / Math.Sin(rho);
                    Ry = q.q3 / Math.Sin(rho);
                    Rz = q.q4 / Math.Sin(rho);
                    objectCluster.Add("Axis Angle A", "CAL", "local", theta);
                    objectCluster.Add("Axis Angle X", "CAL", "local", Rx);
                    objectCluster.Add("Axis Angle Y", "CAL", "local", Ry);
                    objectCluster.Add("Axis Angle Z", "CAL", "local", Rz);
                    objectCluster.Add("Quaternion 0", "CAL", "local", q.q1);
                    objectCluster.Add("Quaternion 1", "CAL", "local", q.q2);
                    objectCluster.Add("Quaternion 2", "CAL", "local", q.q3);
                    objectCluster.Add("Quaternion 3", "CAL", "local", q.q4);
                }
            }
            return objectCluster;
        }


        /**
	 * Converts the raw packet byte values, into the corresponding calibrated and uncalibrated sensor values, the Instruction String determines the output 
	 * @param newPacket a byte array containing the current received packet
	 * @param Instructions an array string containing the commands to execute. It is currently not fully supported
	 * @return
	 */

        protected long[] ParseData(byte[] data, String[] dataType)
        {
            int iData = 0;
            long[] formattedData = new long[dataType.Length];

            for (int i = 0; i < dataType.Length; i++)
                if (dataType[i] == "u8")
                {
                    formattedData[i] = (int)data[iData];
                    iData = iData + 1;
                }
                else if (dataType[i] == "i8")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)0xFF & data[iData]), 8);
                    iData = iData + 1;
                }
                else if (dataType[i] == "u12")
                {
                    formattedData[i] = (int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8));
                    iData = iData + 2;
                }
                else if (dataType[i] == "i12>")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8)), 16);
                    formattedData[i] = formattedData[i] >> 4; // shift right by 4 bits
                    iData = iData + 2;
                }
                else if (dataType[i] == "u16")
                {
                    formattedData[i] = (int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8));
                    iData = iData + 2;
                }
                else if (dataType[i] == "u16r")
                {
                    formattedData[i] = (int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData + 0] & 0xFF) << 8));
                    iData = iData + 2;
                }
                else if (dataType[i] == "i16")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8)), 16);
                    //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
                    iData = iData + 2;
                }
                else if (dataType[i] == "i16*")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8)), 16);
                    //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
                    iData = iData + 2;
                }
                else if (dataType[i] == "i16r")
                {
                    formattedData[i] = Calculatetwoscomplement((int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8)), 16);
                    //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
                    iData = iData + 2;
                }
                else if (dataType[i] == "u24r")
                {
                    long xmsb = ((long)(data[iData + 0] & 0xFF) << 16);
                    long msb = ((long)(data[iData + 1] & 0xFF) << 8);
                    long lsb = ((long)(data[iData + 2] & 0xFF));
                    formattedData[i] = xmsb + msb + lsb;
                    iData = iData + 3;
                }
                else if (dataType[i] == "i24r")
                {
                    long xmsb = ((long)(data[iData + 0] & 0xFF) << 16);
                    long msb = ((long)(data[iData + 1] & 0xFF) << 8);
                    long lsb = ((long)(data[iData + 2] & 0xFF));
                    formattedData[i] = xmsb + msb + lsb;
                    formattedData[i] = Calculatetwoscomplement((int)formattedData[i], 24);
                    iData = iData + 3;
                }
            return formattedData;
        }

        protected int getSignalIndex(String signalName)
        {
            int iSignal = 0; //used to be -1, putting to zero ensure it works eventhough it might be wrong SR30
            for (int i = 0; i < SignalNameArray.Length; i++)
            {
                if (signalName == SignalNameArray[i])
                {
                    iSignal = i;
                }
            }

            return iSignal;
        }

        public void WriteSensors(int sensors)
        {
            if (SensorConflictCheck(sensors))
            {
                if (HardwareVersion != (int)Shimmer.ShimmerVersion.SHIMMER3)
                {
                    SerialPort.Write(new byte[3] { (byte) PacketTypeShimmer2.SET_SENSORS_COMMAND,
                        (byte)(sensors & 0xff), (byte)(sensors>>8 & 0xff)}, 0, 3);

                    if ((sensors & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                    {
                        System.Threading.Thread.Sleep(7000);
                    }
                    else
                    {
                        System.Threading.Thread.Sleep(500);
                    }

                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.INQUIRY_COMMAND }, 0, 1);
                    // give the shimmer a chance to process the previous command (required?)
                    System.Threading.Thread.Sleep(500);
                }
                else
                {
                    byte firstByte = (byte)(sensors & 0xff);
                    byte secondByte = (byte)(sensors >> 8 & 0xff);
                    byte thirdByte = (byte)(sensors >> 16 & 0xff);
                    SerialPort.Write(new byte[4] { (byte)PacketTypeShimmer2.SET_SENSORS_COMMAND, firstByte, secondByte, thirdByte }, 0, 4);
                    System.Threading.Thread.Sleep(1000);
                    
                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.INQUIRY_COMMAND }, 0, 1);
                    // give the shimmer a chance to process the previous command (required?)
                    System.Threading.Thread.Sleep(500);
                }
                ReadCalibrationParameters("All");
            }
        }

        private bool SensorConflictCheck(int enabledSensors)
        {
            bool pass = true;
            if (HardwareVersion != (int)ShimmerVersion.SHIMMER3)
            {
                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                    {
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                    {
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                    {
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                    {
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                    {
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer2.SENSOR_STRAIN_GAUGE) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GYRO) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_MAG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EMG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_ECG) > 0)
                    {
                        pass = false;
                    }
                    else if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    else if (GetVReg() == true)
                    { // if the 5volt reg is set 
                        pass = false;
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A0) > 0)
                {
                    if (((enabledSensors & 0xFFFFF) & (int)SensorBitmapShimmer3.SENSOR_VBATT) > 0)
                    {
                        pass = false;
                    }
                    else if (GetPMux() == true)
                    {
                        WritePMux(0);
                    }
                }

                if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A7) > 0)
                {
                    if (((enabledSensors & 0xFFFFF) & (int)SensorBitmapShimmer3.SENSOR_VBATT) > 0)
                    {
                        pass = false;
                    }
                    else if (GetPMux() == true)
                    {
                        WritePMux(0);
                    }
                }

                if (((enabledSensors & 0xFFFFF) & (int)SensorBitmapShimmer3.SENSOR_VBATT) > 0)
                {
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A7) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF) & (int)SensorBitmapShimmer2.SENSOR_EXP_BOARD_A0) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFF) & (int)SensorBitmapShimmer3.SENSOR_VBATT) > 0)
                    {
                        if (GetPMux() == false)
                        {

                            WritePMux(1);
                        }
                    }
                }
                if (!pass)
                {

                }
            }
            else
            {
                //Shimmer3
                if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                {
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                {
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                }
                if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                {
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                {
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A1) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                    {
                        pass = false;
                    }
                }
                if (((enabledSensors & 0xFF00) & (int)SensorBitmapShimmer3.SENSOR_BRIDGE_AMP) > 0)
                {
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A14) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A12) > 0)
                    {
                        pass = false;
                    }
                    if (((enabledSensors & 0xFFFFFF) & (int)SensorBitmapShimmer3.SENSOR_INT_A13) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_GSR) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_24BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG1_16BIT) > 0)
                    {
                        pass = false;
                    }
                    if ((enabledSensors & (int)SensorBitmapShimmer3.SENSOR_EXG2_16BIT) > 0)
                    {
                        pass = false;
                    }
                }
            }
            return pass;
        }

        public void StartStreaming()
        {
            if (ShimmerState == SHIMMER_STATE_CONNECTED)
            {
                if (ShimmerState != SHIMMER_STATE_STREAMING)
                {
                    KeepObjectCluster = null;
                    SetState(SHIMMER_STATE_STREAMING);
                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.START_STREAMING_COMMAND }, 0, 1);
                }
            }
        }

        public void StopStreaming()
        {
            try
            {
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.STOP_STREAMING_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(200);
                SerialPort.DiscardInBuffer();
                ObjectClusterBuffer.Clear();
                if (SerialPort.IsOpen)
                {
                    SetState(SHIMMER_STATE_CONNECTED);
                }
            }
            catch { };
        }

        public int GetPacketBufferSize()
        {
            return ObjectClusterBuffer.Count;
        }

        public String GetDeviceName()
        {
            return DeviceName;
        }

        public String GetComPort()
        {
            return ComPort;
        }
        public void SetComPort(String comPort){
            ComPort = comPort;
        }

        public int GetShimmerVersion()
        {
            return HardwareVersion;
        }

        public double GetSamplingRate()
        {
            return SamplingRate;
        }

        public int GetEnabledSensors()
        {
            return EnabledSensors;
        }

        public int GetMagRange()
        {
            return MagGain;
        }

        public void SetMagRange(int range)
        {
            MagGain = range;
        }

        public int GetAccelRange()
        {
            return AccelRange;
        }

        public void SetAccelRange(int range)
        {
            AccelRange = range;
        }

        public int GetGyroRange()
        {
            return GyroRange;
        }

        public void SetGyroRange(int range)
        {
            GyroRange = range;
        }

        public int GetGSRRange()
        {
            return GSRRange;
        }

        public void SetGSRRange(int range)
        {
            GSRRange = range;
        }

        public int GetMagSamplingRate()
        {
            return magSamplingRate;
        }

        public void SetMagSamplingRate(int rate)
        {
            magSamplingRate = rate;
        }

        public void SetAccelSamplingRate(int rate)
        {
            AccelSamplingRate = rate;
        }

        public int GetAccelSamplingRate()
        {
            return AccelSamplingRate;
        }

        public int GetInternalExpPower()
        {
            return internalExpPower;
        }

        public void SetInternalExpPower(int value)
        {
            internalExpPower = value;
        }

        public int GetPressureResolution()
        {
            return PressureResolution;
        }

        public void SetPressureResolution(int setting)
        {
            PressureResolution = setting;
        }

        public void SetVReg(bool val)
        {
            if (val)
            {
                ConfigSetupByte0 |= (int)ConfigSetupByte0Bitmap.Config5VReg;
            }
            else
            {
                ConfigSetupByte0 &= ~(int)ConfigSetupByte0Bitmap.Config5VReg;
            }
        }

        public bool GetVReg()
        {
            if ((ConfigSetupByte0 & (int)ConfigSetupByte0Bitmap.Config5VReg) == 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public void SetConfigSetupByte0(int val)
        {
            ConfigSetupByte0 = val;
        }

        public int GetConfigSetupByte0()
        {
            return (int)ConfigSetupByte0;
        }

        public byte[] GetEXG1RegisterContents()
        {
            return Exg1RegArray;
        }

        public byte GetEXG1RegisterByte(int index)
        {
            return Exg1RegArray[index];
        }

        public byte[] GetEXG2RegisterContents()
        {
            return Exg2RegArray;
        }

        public byte GetEXG2RegisterByte(int index)
        {
            return Exg2RegArray[index];
        }

        public void SetEXG1RegisterContents(byte[] exgReg)
        {
            for (int i = 0; i < exgReg.Length; i++)
            {
                Exg1RegArray[i] = exgReg[i];
            }
        }

        public void SetEXG2RegisterContents(byte[] exgReg)
        {
            for (int i = 0; i < exgReg.Length; i++)
            {
                Exg2RegArray[i] = exgReg[i];
            }
        }

        public void SetPMux(bool val)
        {
            if (val)
            {
                ConfigSetupByte0 |= (int)ConfigSetupByte0Bitmap.ConfigPMux;
            }
            else
            {
                ConfigSetupByte0 &= ~(int)ConfigSetupByte0Bitmap.ConfigPMux;
            }
        }

        public bool GetPMux()
        {
            //return pMux;
            if ((ConfigSetupByte0 & (int)ConfigSetupByte0Bitmap.ConfigPMux) == 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public void Set3DOrientation(bool value)
        {
            Orientation3DEnabled = value;
        }

        public bool Is3DOrientationEnabled()
        {
            return Orientation3DEnabled;
        }

        public void SetGyroOnTheFlyCalibration(Boolean enable, int bufferSize, double threshold)
        {
            EnableGyroOnTheFlyCalibration = enable;
            if (enable)
            {
                ListSizeGyroOnTheFly = bufferSize;
                ThresholdGyroOnTheFly = threshold;
            }
        }

        public bool IsGyroOnTheFlyCalEnabled()
        {
            return EnableGyroOnTheFlyCalibration;
        }

        public bool IsLowPowerAccelEnabled()
        {
            return LowPowerAccelEnabled;
        }

        public bool IsLowPowerGyroEnabled()
        {
            return LowPowerGyroEnabled;
        }

        public bool IsLowPowerMagEnabled()
        {
            return LowPowerMagEnabled;
        }

        public string GetFirmwareVersionFullName()
        {
            return FirmwareVersionFullName;
        }

        public double GetFirmwareVersion()
        {
            return FirmwareVersion;
        }

        public int GetFirmwareInternal()
        {
            return FirmwareInternal;
        }

        public int GetCompatibilityCode()
        {
            return CompatibilityCode;
        }

        private void SetCompatibilityCode()
        {
            CompatibilityCode = 0;
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                if (FirmwareIdentifier == 1)    //BtStream
                {
                    if (FirmwareVersion == 0.1)
                    {
                        CompatibilityCode = 1;
                    }
                    else if (FirmwareVersion == 0.2)
                    {
                        CompatibilityCode = 2;
                    }
                    else if (FirmwareVersion == 0.3)
                    {
                        CompatibilityCode = 3;
                    }
                    else if (FirmwareVersion >= 0.4)
                    {
                        CompatibilityCode = 4;
                    }
                }
                else if (FirmwareIdentifier == 3)   //LogAndStream
                {
                    if (FirmwareVersion == 0.1)
                    {
                        CompatibilityCode = 3;
                    }
                    else if (FirmwareVersion >= 0.2)
                    {
                        CompatibilityCode = 4;
                    }
                }
            }
            else
            {
                if (FirmwareIdentifier == 1)
                {
                    if (FirmwareVersion == 1.2)
                    {
                        CompatibilityCode = 1;
                    }
                }
            }
        }

        public void SetState(int state)
        {
            ShimmerState = state;
            EventHandler handler = UICallback;
            if (handler != null)
            {
                CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_STATE_CHANGE, (object)state);
                handler(this, newEventArgs);
            }
        }

        public int GetState()
        {
            return ShimmerState;
        }

        public String GetStateString()
        {
            if (ShimmerState == SHIMMER_STATE_CONNECTED)
            {
                return "Connected";
            }
            else if (ShimmerState == SHIMMER_STATE_CONNECTING)
            {
                return "Connecting";
            }
            else if (ShimmerState == SHIMMER_STATE_STREAMING)
            {
                return "Streaming";
            }
            else
            {
                return "None";
            }
        }

        public double GetPacketReceptionRate()
        {
            return PacketReceptionRate;
        }

        public void ToggleLED()
        {
            if (SerialPort.IsOpen)
            {
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.TOGGLE_LED_COMMAND }, 0, 1);
            }
        }

        public void ReadSamplingRate()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_SAMPLING_RATE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadMagRange()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_MAG_GAIN_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadAccelRange()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_ACCEL_RANGE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadGyroRange()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.GET_MPU9150_GYRO_RANGE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadGSRRange()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_GSR_RANGE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(500);
        }

        public void ReadMagSamplingRate()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_MAG_SAMPLING_RATE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        private void ReadAccelSamplingRate()
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.GET_ACCEL_SAMPLING_RATE_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(200);
            }
        }

        public void ReadAccelCalibration()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_ACCEL_CALIBRATION_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(500);
        }

        public void ReadGyroCalibration()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_GYRO_CALIBRATION_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(500);
        }

        public void ReadMagCalibration()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_MAG_CALIBRATION_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(500);
        }

        public void ReadInternalExpPower()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadBlinkLED()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_BLINK_LED }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadConfigByte0()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_CONFIG_SETUP_BYTE0_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void ReadCalibrationParameters(String sensors)
        {
            if (String.Equals(sensors, "All", StringComparison.Ordinal))
            {
                SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.GET_ALL_CALIBRATION_COMMAND }, 0, 1);
                System.Threading.Thread.Sleep(300);
            }
        }

        public void ReadPressureCalibrationCoefficients()
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                //if (FirmwareVersion > 0.1)
                if (CompatibilityCode > 1)
                {
                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND }, 0, 1);
                    System.Threading.Thread.Sleep(800);
                }
            }
        }

        public void ReadEXGConfigurations(int chipID)
        {
            ChipID = chipID;
            //if ((FirmwareIdentifier == 1 && FirmwareVersion >= 0.3) || FirmwareIdentifier == 3)
            if (CompatibilityCode >= 3)
            {
                if (ChipID == 1 || ChipID == 2)
                {
                    System.Threading.Thread.Sleep(300);
                    SerialPort.Write(new byte[4] { (byte)PacketTypeShimmer3.GET_EXG_REGS_COMMAND, (byte)(ChipID - 1), 0, 10 }, 0, 4);
                    System.Threading.Thread.Sleep(300);
                }
            }
        }

        public void Inquiry()
        {
            SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer2.INQUIRY_COMMAND }, 0, 1);
            System.Threading.Thread.Sleep(200);
        }

        public void WriteSamplingRate(double rate)
        {
            SamplingRate = rate;
            if (!(HardwareVersion == (int)ShimmerVersion.SHIMMER3))
            {
                rate = 1024 / rate; //the equivalent hex setting

                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_SAMPLING_RATE_COMMAND, (byte)Math.Round(rate) }, 0, 2);
            }
            else
            {
                int samplingByteValue = (int)(32768 / rate);
                SerialPort.Write(new byte[3] { (byte)PacketTypeShimmer2.SET_SAMPLING_RATE_COMMAND, (byte)(samplingByteValue & 0xFF), (byte)((samplingByteValue >> 8) & 0xFF) }, 0, 3);
            }
            System.Threading.Thread.Sleep(200);
        }

        public void WriteMagRange(int range)
        {
            if (FirmwareVersionFullName.Equals("BoilerPlate 0.1.0"))
            {
            }
            else
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_MAG_GAIN_COMMAND, (byte)range }, 0, 2);
                System.Threading.Thread.Sleep(200);
                MagGain = range;
            }
        }

        public void WriteGyroRange(int range)
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer3.SET_MPU9150_GYRO_RANGE_COMMAND, (byte)range }, 0, 2);
                System.Threading.Thread.Sleep(200);
                GyroRange = range;
            }
        }

        public void WriteGSRRange(int range)
        {
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_GSR_RANGE_COMMAND, (byte)range }, 0, 2);
            GSRRange = range;
            System.Threading.Thread.Sleep(200);
        }

        public void WriteAccelRange(int range)
        {
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_ACCEL_RANGE_COMMAND, (byte)range }, 0, 2);
            System.Threading.Thread.Sleep(200);
            AccelRange = range;
        }

        public void WritePressureResolution(int setting)
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer3.SET_BMP180_PRES_RESOLUTION_COMMAND, (byte)setting }, 0, 2);
                PressureResolution = setting;
                System.Threading.Thread.Sleep(200);
            }
        }

        private void WriteMagSamplingRate(int rate)
        {
            if (FirmwareVersionFullName.Equals("BoilerPlate 0.1.0"))
            {
            }
            else
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_MAG_SAMPLING_RATE_COMMAND, (byte)rate }, 0, 2);
                magSamplingRate = rate;
                System.Threading.Thread.Sleep(200);
            }
        }

        private void WriteWRAccelSamplingRate(int rate)
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer3.SET_ACCEL_SAMPLING_RATE_COMMAND, (byte)rate }, 0, 2);
                AccelSamplingRate = rate;
                System.Threading.Thread.Sleep(200);
            }
        }

        private void WriteGyroSamplingRate(int rate)
        {
            if (HardwareVersion == (int)ShimmerVersion.SHIMMER3)
            {
                SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer3.SET_MPU9150_SAMPLING_RATE_COMMAND, (byte)rate }, 0, 2);
                Mpu9150SamplingRate = rate;
                System.Threading.Thread.Sleep(200);
            }
        }

        public void WriteInternalExpPower(int expPower)
        {
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer3.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND, (byte)expPower }, 0, 2);
            System.Threading.Thread.Sleep(200);
            internalExpPower = expPower;
        }

        public void Write5VReg(int value)
        {
            if (value == 1)
            {
                SetVReg(true);
            }
            else
            {
                SetVReg(false);
            }
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_5V_REGULATOR_COMMAND, (byte)value }, 0, 2);
            System.Threading.Thread.Sleep(200);
        }

        public void WritePMux(int value)
        {
            if (value == 1)
            {
                SetPMux(true);
            }
            else
            {
                SetPMux(false);
            }
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_POWER_MUX_COMMAND, (byte)value }, 0, 2);
            System.Threading.Thread.Sleep(200);
        }

        public void WriteBufferSize(int size)
        {
            SerialPort.Write(new byte[2] { (byte)PacketTypeShimmer2.SET_BUFFER_SIZE_COMMAND, (byte)size }, 0, 2);
            System.Threading.Thread.Sleep(200);
        }

        public void SetLowPowerMag(bool enable)
        {
            LowPowerMagEnabled = enable;
            if (HardwareVersion != (int)ShimmerVersion.SHIMMER3)
            {
                if (!LowPowerMagEnabled)
                {
                    if (SamplingRate >= 50)
                    {
                        WriteMagSamplingRate(6);
                    }
                    else if (SamplingRate >= 20)
                    {
                        WriteMagSamplingRate(5);
                    }
                    else if (SamplingRate >= 10)
                    {
                        WriteMagSamplingRate(4);
                    }
                    else
                    {
                        WriteMagSamplingRate(3);
                    }
                }
                else
                {
                    WriteMagSamplingRate(4);
                }
            }
            else
            {
                if (!LowPowerMagEnabled)
                {
                    if (SamplingRate <= 1)
                    {
                        WriteMagSamplingRate(1);
                    }
                    else if (SamplingRate <= 15)
                    {
                        WriteMagSamplingRate(4);
                    }
                    else if (SamplingRate <= 30)
                    {
                        WriteMagSamplingRate(5);
                    }
                    else if (SamplingRate <= 75)
                    {
                        WriteMagSamplingRate(6);
                    }
                    else
                    {
                        WriteMagSamplingRate(7);
                    }
                }
                else
                {
                    if (SamplingRate >= 10)
                    {
                        WriteMagSamplingRate(4);
                    }
                    else
                    {
                        WriteMagSamplingRate(1);
                    }
                }
            }
        }

        public void SetLowPowerGyro(bool enable)
        {
            LowPowerGyroEnabled = enable;
            if (!LowPowerGyroEnabled)
            {
                if (SamplingRate <= 51.28)
                {
                    WriteGyroSamplingRate(0x9B);
                }
                else if (SamplingRate <= 102.56)
                {
                    WriteGyroSamplingRate(0x4D);
                }
                else if (SamplingRate <= 129.03)
                {
                    WriteGyroSamplingRate(0x3D);
                }
                else if (SamplingRate <= 173.91)
                {
                    WriteGyroSamplingRate(0x2D);
                }
                else if (SamplingRate <= 205.13)
                {
                    WriteGyroSamplingRate(0x26);
                }
                else if (SamplingRate <= 258.06)
                {
                    WriteGyroSamplingRate(0x1E);
                }
                else if (SamplingRate <= 533.33)
                {
                    WriteGyroSamplingRate(0xE);
                }
                else
                {
                    WriteGyroSamplingRate(6);
                }
            }
            else
            {
                WriteGyroSamplingRate(0xFF);
            }
        }

        public void SetLowPowerAccel(bool enable)
        {
            LowPowerAccelEnabled = enable;
            if (!LowPowerAccelEnabled)
            {
                //enableLowResolutionMode(false);
                if (SamplingRate <= 1)
                {
                    WriteWRAccelSamplingRate(1);
                }
                else if (SamplingRate <= 10)
                {
                    WriteWRAccelSamplingRate(2);
                }
                else if (SamplingRate <= 25)
                {
                    WriteWRAccelSamplingRate(3);
                }
                else if (SamplingRate <= 50)
                {
                    WriteWRAccelSamplingRate(4);
                }
                else if (SamplingRate <= 100)
                {
                    WriteWRAccelSamplingRate(5);
                }
                else if (SamplingRate <= 200)
                {
                    WriteWRAccelSamplingRate(6);
                }
                else
                {
                    WriteWRAccelSamplingRate(7);
                }
            }
            else
            {
                //enableLowResolutionMode(true);
                WriteWRAccelSamplingRate(2);
            }
        }

        public bool IsDefaultECGConfigurationEnabled()
        {
            bool enabled = false;
            bool byte0Valid = false;    //byte 0 valid for 0 -> 6
            bool byte10Valid = false;   //byte 10 valid for 0 -> 6

            if ((Exg1RegArray[0] >> 3) == 0)
            {
                byte0Valid = true;
            }
            if ((Exg2RegArray[0] >> 3) == 0)
            {
                byte10Valid = true;
            }

            if (byte0Valid && (Exg1RegArray[1] == 160) && (Exg1RegArray[2] == 16) && (Exg1RegArray[3] == 64)
                && (Exg1RegArray[4] == 64) && (Exg1RegArray[5] == 45) && (Exg1RegArray[6] == 0) && (Exg1RegArray[7] == 0)
                && (Exg1RegArray[8] == 2) && (Exg1RegArray[9] == 3) && byte10Valid && (Exg2RegArray[1] == 160)
                && (Exg2RegArray[2] == 16) && (Exg2RegArray[3] == 64) && (Exg2RegArray[4] == 71) && (Exg2RegArray[5] == 0)
                && (Exg2RegArray[6] == 0) && (Exg2RegArray[7] == 0) && (Exg2RegArray[8] == 2) && (Exg2RegArray[9] == 1))
            {
                enabled = true;
            }
            return enabled;
        }

        public bool IsDefaultEMGConfigurationEnabled()
        {
            bool enabled = false;
            bool byte0Valid = false;    //byte 0 valid for 0 -> 6 
            bool byte10Valid = false;   //byte 10 valid for 0 -> 6

            if ((Exg1RegArray[0] >> 3) == 0)
            {
                byte0Valid = true;
            }
            if ((Exg2RegArray[0] >> 3) == 0)
            {
                byte10Valid = true;
            }

            if (byte0Valid && (Exg1RegArray[1] == 160) && (Exg1RegArray[2] == 16) && (Exg1RegArray[3] == 105)
                && (Exg1RegArray[4] == 96) && (Exg1RegArray[5] == 0) && (Exg1RegArray[6] == 0) && (Exg1RegArray[7] == 0)
                && (Exg1RegArray[8] == 2) && (Exg1RegArray[9] == 3) && byte10Valid && (Exg2RegArray[1] == 160)
                && (Exg2RegArray[2] == 16) && (Exg2RegArray[3] == 129) && (Exg2RegArray[4] == 129) && (Exg2RegArray[5] == 0)
                && (Exg2RegArray[6] == 0) && (Exg2RegArray[7] == 0) && (Exg2RegArray[8] == 2) && (Exg2RegArray[9] == 1))
            {
                enabled = true;
            }
            return enabled;
        }

        protected int ConvertEXGGainSettingToValue(int setting)
        {
            if (setting == 0)
            {
                return 6;
            }
            else if (setting == 1)
            {
                return 1;
            }
            else if (setting == 2)
            {
                return 2;
            }
            else if (setting == 3)
            {
                return 3;
            }
            else if (setting == 4)
            {
                return 4;
            }
            else if (setting == 5)
            {
                return 8;
            }
            else if (setting == 6)
            {
                return 12;
            }
            else
            {
                return -1; // -1 means invalid value
            }
        }

        public void WriteEXGConfigurations(byte[] valuesChip1, byte[] valuesChip2)
        {
            if (SerialPort.IsOpen)
            {
                if (CompatibilityCode >= 3)
                {
                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.SET_EXG_REGS_COMMAND }, 0, 1);
                    SerialPort.Write(new byte[1] { (byte)0 }, 0, 1); //CHIPID1
                    SerialPort.Write(new byte[1] { (byte)0 }, 0, 1);
                    SerialPort.Write(new byte[1] { (byte)10 }, 0, 1);
                    for (int i = 0; i < 10; i++)
                    {
                        SerialPort.Write(new byte[1] { valuesChip1[i] }, 0, 1);
                        Exg1RegArray[i] = valuesChip1[i];
                    }

                    System.Threading.Thread.Sleep(500);

                    SerialPort.Write(new byte[1] { (byte)PacketTypeShimmer3.SET_EXG_REGS_COMMAND }, 0, 1);
                    SerialPort.Write(new byte[1] { (byte)1 }, 0, 1); //CHIPID2
                    SerialPort.Write(new byte[1] { (byte)0 }, 0, 1);
                    SerialPort.Write(new byte[1] { (byte)10 }, 0, 1);
                    for (int i = 0; i < 10; i++)
                    {
                        SerialPort.Write(new byte[1] { valuesChip2[i] }, 0, 1);
                        Exg2RegArray[i] = valuesChip2[i];
                    }

                    System.Threading.Thread.Sleep(500);
                }
            }
        }

        protected double CalibrateTimeStamp(double timeStamp)
        {
            //first convert to continuous time stamp
            double calibratedTimeStamp = 0;
            if (LastReceivedTimeStamp > (timeStamp + (65536 * CurrentTimeStampCycle)))
            {
                CurrentTimeStampCycle = CurrentTimeStampCycle + 1;
            }

            LastReceivedTimeStamp = (timeStamp + (65536 * CurrentTimeStampCycle));
            calibratedTimeStamp = LastReceivedTimeStamp / 32768 * 1000;   // to convert into mS
            if (FirstTimeCalTime)
            {
                FirstTimeCalTime = false;
                CalTimeStart = calibratedTimeStamp;
            }
            if (LastReceivedCalibratedTimeStamp != -1)
            {
                double timeDifference = calibratedTimeStamp - LastReceivedCalibratedTimeStamp;
                double clockConstant = 1024;
                if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2R || HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER2)
                {
                    clockConstant = 1024;
                }
                else if (HardwareVersion == (int)Shimmer.ShimmerVersion.SHIMMER3)
                {
                    clockConstant = 32768;
                }

                if (timeDifference > (1 / ((clockConstant / ADCRawSamplingRateValue) - 1)) * 1000)
                {
                    PacketLossCount = PacketLossCount + 1;
                    long mTotalNumberofPackets = (long)((calibratedTimeStamp - CalTimeStart) / (1 / (clockConstant / ADCRawSamplingRateValue) * 1000));
                    PacketReceptionRate = (double)((mTotalNumberofPackets - PacketLossCount) / (double)mTotalNumberofPackets) * 100;

                    if (PacketReceptionRate < 99)
                    {
                        //System.Console.WriteLine("PRR: " + PacketReceptionRate);
                    }
                }
            }

            EventHandler handler = UICallback;
            if (handler != null)
            {
                CustomEventArgs newEventArgs = new CustomEventArgs((int)ShimmerIdentifier.MSG_IDENTIFIER_PACKET_RECEPTION_RATE, (object)GetPacketReceptionRate());
                handler(this, newEventArgs);
            }

            LastReceivedCalibratedTimeStamp = calibratedTimeStamp;
            return calibratedTimeStamp - CalTimeStart; // make it start at zero
        }

        protected double CalibrateGsrData(double gsrUncalibratedData, double p1, double p2)
        {
            gsrUncalibratedData = (double)((int)gsrUncalibratedData & 4095);
            //the following polynomial is deprecated and has been replaced with a more accurate linear one, see GSR user guide for further details
            //double gsrCalibratedData = (p1*Math.pow(gsrUncalibratedData,4)+p2*Math.pow(gsrUncalibratedData,3)+p3*Math.pow(gsrUncalibratedData,2)+p4*gsrUncalibratedData+p5)/1000;
            //the following is the new linear method see user GSR user guide for further details
            double gsrCalibratedData = (1 / (p1 * gsrUncalibratedData + p2)) * 1000; //kohms 
            return gsrCalibratedData;
        }

        protected double[] CalibratePressureSensorData(double UP, double UT)
        {
            double X1 = (UT - AC6) * AC5 / 32768;
            double X2 = (MC * 2048 / (X1 + MD));
            double B5 = X1 + X2;
            double T = (B5 + 8) / 16;

            double B6 = B5 - 4000;
            X1 = (B2 * (Math.Pow(B6, 2) / 4096)) / 2048;
            X2 = AC2 * B6 / 2048;
            double X3 = X1 + X2;
            double B3 = (((AC1 * 4 + X3) * (1 << PressureResolution) + 2)) / 4;
            X1 = AC3 * B6 / 8192;
            X2 = (B1 * (Math.Pow(B6, 2) / 4096)) / 65536;
            X3 = ((X1 + X2) + 2) / 4;
            double B4 = AC4 * (X3 + 32768) / 32768;
            double B7 = (UP - B3) * (50000 >> PressureResolution);
            double p = 0;
            if (B7 < 2147483648L)
            { //0x80000000
                p = (B7 * 2) / B4;
            }
            else
            {
                p = (B7 / B4) * 2;
            }
            X1 = ((p / 256.0) * (p / 256.0) * 3038) / 65536;
            X2 = (-7357 * p) / 65536;
            p = p + ((X1 + X2 + 3791) / 16);

            double[] caldata = new double[2];
            caldata[0] = p;
            caldata[1] = T / 10;
            return caldata;
        }

        protected double CalibrateU12AdcValue(double uncalibratedData, double offset, double vRefP, double gain)
        {
            double calibratedData = (uncalibratedData - offset) * (((vRefP * 1000) / gain) / 4095);
            return calibratedData;
        }

        protected double[] CalibrateInertialSensorData(double[] data, double[,] AM, double[,] SM, double[,] OV)
        {
            /*  Based on the theory outlined by Ferraris F, Grimaldi U, and Parvis M.  
               in "Procedure for effortless in-field calibration of three-axis rate gyros and accelerometers" Sens. Mater. 1995; 7: 311-30.            
               C = [R^(-1)] .[K^(-1)] .([U]-[B])
                where.....
                [C] -> [3 x n] Calibrated Data Matrix 
                [U] -> [3 x n] Uncalibrated Data Matrix
                [B] ->  [3 x n] Replicated Sensor Offset Vector Matrix 
                [R^(-1)] -> [3 x 3] Inverse Alignment Matrix
                [K^(-1)] -> [3 x 3] Inverse Sensitivity Matrix
                n = Number of Samples
                */
            double[] tempdata = data;
            double[,] data2d = new double[3, 1];
            data2d[0, 0] = data[0];
            data2d[1, 0] = data[1];
            data2d[2, 0] = data[2];
            data2d = MatrixMultiplication(MatrixMultiplication(MatrixInverse3x3(AM), MatrixInverse3x3(SM)), MatrixMinus(data2d, OV));
            tempdata[0] = data2d[0, 0];
            tempdata[1] = data2d[1, 0];
            tempdata[2] = data2d[2, 0];
            return tempdata;
        }

        private double[,] MatrixMultiplication(double[,] a, double[,] b)
        {

            int aRows = a.GetLength(0),
                aColumns = a.GetLength(1),
                 bRows = b.GetLength(0),
                 bColumns = b.GetLength(1);
            double[,] resultant = new double[aRows, bColumns];

            for (int i = 0; i < aRows; i++)
            { // aRow
                for (int j = 0; j < bColumns; j++)
                { // bColumn
                    for (int k = 0; k < aColumns; k++)
                    { // aColumn
                        resultant[i, j] += a[i, k] * b[k, j];
                    }
                }
            }

            return resultant;
        }

        private double[,] MatrixInverse3x3(double[,] data)
        {
            double a, b, c, d, e, f, g, h, i;
            a = data[0, 0];
            b = data[0, 1];
            c = data[0, 2];
            d = data[1, 0];
            e = data[1, 1];
            f = data[1, 2];
            g = data[2, 0];
            h = data[2, 1];
            i = data[2, 2];
            //
            double deter = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;
            double[,] answer = new double[3, 3];
            answer[0, 0] = (1 / deter) * (e * i - f * h);

            answer[0, 1] = (1 / deter) * (c * h - b * i);
            answer[0, 2] = (1 / deter) * (b * f - c * e);
            answer[1, 0] = (1 / deter) * (f * g - d * i);
            answer[1, 1] = (1 / deter) * (a * i - c * g);
            answer[1, 2] = (1 / deter) * (c * d - a * f);
            answer[2, 0] = (1 / deter) * (d * h - e * g);
            answer[2, 1] = (1 / deter) * (g * b - a * h);
            answer[2, 2] = (1 / deter) * (a * e - b * d);
            return answer;
        }

        private double[,] MatrixMinus(double[,] a, double[,] b)
        {

            int aRows = a.GetLength(0),
            aColumns = a.GetLength(1),
            bRows = b.GetLength(0),
            bColumns = b.GetLength(1);
            double[,] resultant = new double[aRows, bColumns];
            for (int i = 0; i < aRows; i++)
            { // aRow
                for (int k = 0; k < aColumns; k++)
                { // aColumn
                    resultant[i, k] = a[i, k] - b[i, k];
                }
            }
            return resultant;
        }

        private double GetStandardDeviation(List<double> doubleList)
        {
            double average = doubleList.Average();
            double sumOfDerivation = 0;
            foreach (double value in doubleList)
            {
                sumOfDerivation += (value) * (value);
            }
            double sumOfDerivationAverage = sumOfDerivation / doubleList.Count;
            return Math.Sqrt(sumOfDerivationAverage - (average * average));
        }

        public void ClearPacketBuffer()
        {
            ObjectClusterBuffer.Clear();
        }

    }

    public class CustomEventArgs : EventArgs
    {
        private int EventIdentifier;
        private object Object;

        public CustomEventArgs(int indicator, object obj)
        {
            this.EventIdentifier = indicator;
            this.Object = obj;
        }

        public int getIndicator()
        {
            return EventIdentifier;
        }

        public object getObject()
        {
            return Object;
        }
    }

}

