//  File:   main.cpp
//  Performs initialization then starts the print engine and event handler
//
//  This file is part of the Ember firmware.
//
//  Copyright 2015 Autodesk, Inc. <http://ember.autodesk.com/>
//    
//  Authors:
//  Richard Greene
//  Jason Lefley
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  THIS PROGRAM IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL,
//  BUT WITHOUT ANY WARRANTY; WITHOUT EVEN THE IMPLIED WARRANTY OF
//  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  SEE THE
//  GNU GENERAL PUBLIC LICENSE FOR MORE DETAILS.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <fstream> 
#include <string>
#include <utils.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdexcept>
#include <Magick++.h>

#include <PrintEngine.h>
#include <EventHandler.h>
#include <TerminalUI.h>
#include <Logger.h>
#include <NetworkInterface.h>
#include <CommandInterpreter.h>
#include <Settings.h>
#include <MessageStrings.h>
#include <Hardware.h>
#include <MotorController.h>
#include <Filenames.h>

#include "StandardIn.h"
#include "CommandPipe.h"
#include "PrinterStatusQueue.h"
#include "Timer.h"
#include "I2C_Resource.h"
#include "GPIO_Interrupt.h"
#include "GPIO.h"
#include "Signals.h"
#include "UdevMonitor.h"
#include "I2C_Device.h"
#include "Projector.h"
#include "HardwareFactory.h"

using namespace std;

// command line argument to suppress use of stdin & stdout
constexpr const char* NO_STDIO = "--nostdio";

// for setting DMA priority to avoid video flicker
constexpr unsigned long MAP_SIZE = 4096UL;
constexpr unsigned long MAP_MASK = (MAP_SIZE - 1);
constexpr off_t REG_PR_OLD_COUNT = 0x4c000054;
constexpr unsigned long PR_OLD_COUNT_VALUE = 0x00FFFFF10;

int main(int argc, char** argv) 
{
    try
    {
        // sets up signal handling
        Signals signals;
        
       //lifei  Magick::InitializeMagick("");
       //lifei  
       //lifei  // see if we should support keyboard input and TerminalUI output
       //lifei  bool useStdio = true;
       //lifei  if (argc > 1) 
       //lifei  {
       //lifei      useStdio = strcmp(argv[1], NO_STDIO) != 0;
       //lifei  }
       //lifei  
       //lifei  // report the firmware version, board serial number, and startup message
       //lifei  Logger::LogMessage(LOG_INFO, PRINTER_STARTUP_MSG);
       //lifei  string version = GetFirmwareVersion();
       //lifei  string fwVersion = string(FW_VERSION_MSG) + version;
       //lifei  Logger::LogMessage(LOG_INFO, fwVersion.c_str());
       //lifei  string serNum = string(BOARD_SER_NUM_MSG) + GetBoardSerialNum();
       //lifei  Logger::LogMessage(LOG_INFO, serNum.c_str());
       //lifei 
       //lifei  if (useStdio)
       //lifei  {
       //lifei      cout << PRINTER_STARTUP_MSG << endl;
       //lifei      cout << fwVersion << std::endl << serNum << std::endl;
       //lifei  }
       //lifei    
       //lifei  // turn on fans
       //lifei  GPIO fan1GPIO(FAN_1_PIN);
       //lifei  GPIO fan2GPIO(FAN_2_PIN);
       //lifei  GPIO fan3GPIO(FAN_3_PIN);

       //lifei  fan1GPIO.SetDirectionOut();
       //lifei  fan2GPIO.SetDirectionOut();
       //lifei  fan3GPIO.SetDirectionOut();

       //lifei  fan1GPIO.SetOutputHigh();
       //lifei  fan2GPIO.SetOutputHigh();
       //lifei  fan3GPIO.SetOutputHigh();
       //lifei  
       //lifei  // prevent video flickering by tweaking the value of REG_PR_OLD_COUNT
       //lifei  // see https://groups.google.com/forum/#!msg/beagleboard/GjxRGeLdmRw/dx-bOXBPBgAJ
       //lifei  // and http://www.lartmaker.nl/lartware/port/devmem2.c 
       //lifei  int fd = open(MEMORY_DEVICE, O_RDWR | O_SYNC);
       //lifei  if(fd < 0)
       //lifei  {
       //lifei      Logger::LogError(LOG_ERR, errno, CantOpenMemoryDevice);
       //lifei      return 1;
       //lifei  }

       //lifei  // map one page 
       //lifei  void* mapBase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, 
       //lifei                       fd, REG_PR_OLD_COUNT & ~MAP_MASK);
       //lifei  if(mapBase == MAP_FAILED)
       //lifei  {
       //lifei      Logger::LogError(LOG_ERR, errno,CantMapPriorityRegister);
       //lifei      return 1;
       //lifei  }

       //lifei  void* addr = ((char*)mapBase) + (REG_PR_OLD_COUNT & MAP_MASK);

       //lifei  *((unsigned long *) addr) = PR_OLD_COUNT_VALUE;

       //lifei  if(munmap(mapBase, MAP_SIZE) < 0)
       //lifei  {
       //lifei      Logger::LogError(LOG_ERR, errno, CantUnMapPriorityRegister);
       //lifei      return 1;
       //lifei  }

       //lifei  close(fd);        
       //lifei  
       //lifei  Settings& settings = PrinterSettings::Instance();
       //lifei  
       //lifei  // If we're upgrading to higher version or downgrading to a lower one,
       //lifei  // update selected printer settings with current default values.
       //lifei  if (version != settings.GetString(FW_VERSION))
       //lifei  {
       //lifei      // update FirmwareVersion setting so that this version of the 
       //lifei      // firmware won't run this code again
       //lifei      settings.Set(FW_VERSION, version);
       //lifei      // restore these settings to their new defaults
       //lifei      cout << UPDATING_DEFAULTS_MSG << endl; 
       //lifei      vector<const char*> intSettings = {Z_HOMING_SPEED, 
       //lifei                                         Z_START_PRINT_SPEED};
       //lifei      for(int i = 0; i < intSettings.size(); i++)
       //lifei      {
       //lifei          settings.Restore(intSettings[i]);
       //lifei          cout << "\t" << intSettings[i] 
       //lifei               << "\t" << settings.GetInt(intSettings[i]) << endl; 
       //lifei      }
       //lifei      
       //lifei      vector<const char*> doubleSettings = {LAYER_OVERHEAD};
       //lifei      for(int i = 0; i < doubleSettings.size(); i++)
       //lifei      {
       //lifei          settings.Restore(doubleSettings[i]);
       //lifei          cout << "\t" << doubleSettings[i] 
       //lifei               << "\t" << settings.GetDouble(doubleSettings[i]) << endl; 
       //lifei      }
       //lifei  }
             
        // ensure directories exist
        //lifei MakePath(settings.GetString(PRINT_DATA_DIR));
        //lifei MakePath(settings.GetString(DOWNLOAD_DIR));
        //lifei MakePath(settings.GetString(STAGING_DIR));

        // create the motor controller
        //I2C_DevicePtr pMotorControllerI2cDevice =
        //        HardwareFactory::CreateMotorControllerI2cDevice();
        //Motor motor(*pMotorControllerI2cDevice);
       
        //// create the front panel
        //I2C_DevicePtr pFrontPanelI2cDevice =
        //        HardwareFactory::CreateFrontPanelI2cDevice();
        //FrontPanel frontPanel(*pFrontPanelI2cDevice); 

        // create the projector
        //lifei I2C_Device projectorI2cDevice(PROJECTOR_SLAVE_ADDRESS,
               //lifei  I2C0_PORT);
        I2C_Device projectorI2cDevice(0x1a,
                2);
        Projector projector(projectorI2cDevice);
	projector.TurnLEDOn();
	projector.ShowWhite();

        //lifei EventHandler eh;

        //lifei StandardIn standardIn;
        //lifei CommandPipe commandPipe;
        //lifei PrinterStatusQueue printerStatusQueue;
        //lifei Timer exposureTimer;
        //lifei Timer temperatureTimer;
        //lifei Timer delayTimer;
        //lifei GPIO_Interrupt doorSensorGPIOInterrupt(DOOR_SENSOR_PIN,
        //lifei         GPIO_INTERRUPT_EDGE_BOTH);
        //lifei GPIO_Interrupt rotationSensorGPIOInterrupt(ROTATION_SENSOR_PIN,
        //lifei         GPIO_INTERRUPT_EDGE_FALLING);
        //lifei UdevMonitor usbDriveConnectionMonitor(UDEV_SUBSYSTEM_BLOCK,
        //lifei         UDEV_DEVTYPE_PARTITION, UDEV_ACTION_ADD);
        //lifei UdevMonitor usbDriveDisconnectionMonitor(UDEV_SUBSYSTEM_BLOCK,
        //lifei         UDEV_DEVTYPE_PARTITION, UDEV_ACTION_REMOVE);
        //lifei 
        //lifei Timer motorTimeoutTimer;
        //lifei I2C_Resource motorControllerTimeout(motorTimeoutTimer,
        //lifei         *pMotorControllerI2cDevice, MC_STATUS_REG);
        //lifei 
        //lifei ResourcePtr pMotorControllerInterruptResource =
        //lifei         HardwareFactory::CreateMotorControllerInterruptResource();
        //lifei I2C_Resource motorControllerInterrupt(*pMotorControllerInterruptResource, 
        //lifei         *pMotorControllerI2cDevice, MC_STATUS_REG);
       
        //lifei ResourcePtr pFrontPanelInterruptResource =
        //lifei         HardwareFactory::CreateFrontPanelInterruptResource();
        //lifei I2C_Resource buttonInterrupt(*pFrontPanelInterruptResource,
        //lifei         *pFrontPanelI2cDevice, BTN_STATUS);

        //lifei eh.AddEvent(Keyboard, &standardIn);
        //lifei eh.AddEvent(UICommand, &commandPipe);
        //lifei eh.AddEvent(PrinterStatusUpdate, &printerStatusQueue);
        //lifei eh.AddEvent(ExposureEnd, &exposureTimer);
        //lifei eh.AddEvent(TemperatureTimer, &temperatureTimer);
        //lifei eh.AddEvent(DelayEnd, &delayTimer);
        //lifei eh.AddEvent(DoorInterrupt, &doorSensorGPIOInterrupt);
        //lifei eh.AddEvent(RotationInterrupt, &rotationSensorGPIOInterrupt);
        //lifei eh.AddEvent(Signal, &signals);
        //lifei eh.AddEvent(USBDriveConnected, &usbDriveConnectionMonitor);
        //lifei eh.AddEvent(USBDriveDisconnected, &usbDriveDisconnectionMonitor);
        //lifei eh.AddEvent(MotorTimeout, &motorControllerTimeout);
        //lifei eh.AddEvent(MotorInterrupt, &motorControllerInterrupt);
        //lifei eh.AddEvent(ButtonInterrupt, &buttonInterrupt);

        // create a print engine that communicates with actual hardware
//lifei        PrintEngine pe(true, motor, projector, printerStatusQueue, exposureTimer,
 //lifei               temperatureTimer, delayTimer, motorTimeoutTimer);
        
        // give it to the settings singleton as an error handler
        //lifei settings.SetErrorHandler(&pe);

        // set the screensaver time, or disable screen saver if demo mode is 
        // being requested via a button press at startup
        //lifei frontPanel.SetAwakeTime(pe.DemoModeRequested() ?
               //lifei  0 : settings.GetInt(FRONT_PANEL_AWAKE_TIME));
    
        // subscribe logger first, so that it will show 
        // its output in the logs ahead of any other subscribers that actually 
        // act on those events
        //lifei Logger logger;
        //lifei eh.Subscribe(PrinterStatusUpdate, &logger);
        //lifei eh.Subscribe(MotorInterrupt, &logger);
        //lifei eh.Subscribe(ButtonInterrupt, &logger);
        //lifei eh.Subscribe(DoorInterrupt, &logger);
        //lifei if (useStdio)
        //lifei     eh.Subscribe(Keyboard, &logger);
        //lifei eh.Subscribe(UICommand, &logger);
        
        // subscribe the print engine to interrupt events
        //lifei eh.Subscribe(MotorInterrupt, &pe);
        //lifei eh.Subscribe(ButtonInterrupt, &pe); 
        //lifei eh.Subscribe(DoorInterrupt, &pe);
        //lifei eh.Subscribe(RotationInterrupt, &pe);
        //lifei 
        //lifei // subscribe the print engine to timer events
        //lifei eh.Subscribe(DelayEnd, &pe);
        //lifei eh.Subscribe(ExposureEnd, &pe);
        //lifei eh.Subscribe(TemperatureTimer, &pe);
        //lifei eh.Subscribe(MotorTimeout, &pe);

        //lifei // subscribe the print engine to the usb addition/removal events
        //lifei eh.Subscribe(USBDriveConnected, &pe);
        //lifei eh.Subscribe(USBDriveDisconnected, &pe);
        //lifei 
        //lifei CommandInterpreter peCmdInterpreter(&pe);
        // subscribe the command interpreter to command input events,
        // from UI and possibly the keyboard
        //lifei eh.Subscribe(UICommand, &peCmdInterpreter); 
        //lifei if (useStdio)
        //lifei     eh.Subscribe(Keyboard, &peCmdInterpreter);   
        //lifei 
        //lifei // subscribe the front panel to printer status events
        //lifei eh.Subscribe(PrinterStatusUpdate, &frontPanel);
      
        // connect the event handler to itself via another command interpreter
        // to allow the event handler to stop when it receives an exit command
        // Also subscribe to Signal event to handle TERM and INT signals
        //lifei CommandInterpreter ehCmdInterpreter(&eh);
        //lifei eh.Subscribe(UICommand, &ehCmdInterpreter);
        //lifei eh.Subscribe(Keyboard, &ehCmdInterpreter);
        //lifei eh.Subscribe(Signal, &eh);
        //lifei 
        //lifei // also connect a network interface, subscribed to printer status events
        //lifei NetworkInterface networkIF;
        //lifei eh.Subscribe(PrinterStatusUpdate, &networkIF);
        //lifei 
        //lifei if (useStdio)
        //lifei {
        //lifei     // also connect a terminal UI, subscribed to printer status events
        //lifei     TerminalUI terminal;
        //lifei     eh.Subscribe(PrinterStatusUpdate, &terminal);
        //lifei }
        
        // start the print engine's state machine
        //pe.Begin();

        // begin handling events
        //eh.Begin();

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}

