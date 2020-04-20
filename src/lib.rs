//! A platform agnostic driver for the AT42QT2120 touch sensor IC in Rust.
//! It is based on the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits
//!
//! ## The device 
//!
//! The AT42QT2120 is a 12 channel QTouch touch sensor that also supports a touch sliders
//!
//! ## Usage
//!
//! Import this crate and an embedded_hal implementation, for example, on an stm32f103 device:
//! 
//! ```
//! extern crate stm32f1xx_hal as hal;
//! extern crate AT42QT2120;
//! ```
//!
//! The AT42QT2120 only has one address option, so to instantiate a device:
//! ```
//! let i2c2 = BlockingI2c::i2c2(..);
//! let mut touch_sensor = At42qt2120::new(i2c2);
//! ```
//! 
//! The AT42QT2120, on reset, has all 12 channels setup as touch input with sane settings. 
//! If only touch buttons are needed, no extra setup is needed.
//! 
//! To enable a slider: 
//! ```
//! let use_wheel = false;
//! let enable_slider = true;
//! touch_sensor.setup_slider(use_wheel, enable_slider); 
//! ```
//! 
//! If a key needs to be disabled, or the threshold changed:
//! ```
//! let key_to_setup = 3;
//! let key_threshold = 20;  //must be 1 or more!
//! let enable_key = true;
//! setup_key(key_to_setup, key_threshold, enable_key);
//! ```
//! 
//! 
//! Reading keys and the slider:
//! ```
//! if touch_sensor.keys_pressed()? {
//!   let keys_pressed = touch_sensor.read_keys()?;
//! }
//!
//! if touch_sensor.slider_pressed()? {
//!   let keys_pressed = touch_sensor.read_slider()?;
//! }
//! ```
//! 
//! individual keys can also be read using:
//! ```
//! let key_to_read = 3;
//! touch_sensor.read_key(key_to_read)?;
//! ```
//! 
//! 
//! ## supported features
//! 
//! Basic features are supported:
//! setting up and reading the slider/wheel
//! setting up and reading keys.
//! Calibration.
//! 
//! Not supported is:
//! setting power mode, changing drift compensation and other advanced settings
//! Grouping keys
//! Setting oversampling on keys
//! 
//! This driver is tested on an stm32f103 and made by someone who is new to rust, ymmv :)


#![no_std]
#![allow(dead_code)]

extern crate embedded_hal as hal;
use hal::blocking::i2c;

/// Driver for the AT42QT2120
pub struct At42qt2120<I2C> {
    i2c: I2C,
    address: u8,
}

pub enum Error<E> {
  /// I2C bus error
  I2c(E), 
  /// A device ID is read that does not match the AT42QT2120
  WrongDeviceID,
  /// threshold must be above 0.
  InvalidThreshold,
}

/// Register defines
#[allow(dead_code)]
#[allow(non_camel_case_types)]
enum Register {
    CHIP_ID = 0x00,
	STATUS = 0x02,
	KEY_STATUS1 = 0x03,
	KEY_STATUS2 = 0x04,
	SLIDER_POSITION = 0x05,
	CALIBRATE = 0x06,
	RESET = 0x07,
	DETECTION_INTEGRATOR = 0x0B,
	SLIDER_OPTIONS = 0x0E,
	KEY_THRESHOLD_START = 0x10,
	KEY_CONTROL_START = 0x1C,
	KEY_SIGNAL_START = 0x34,
}

impl<I2C, E> At42qt2120<I2C>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
	{
	/// initialize the AT42QT2120 driver.
	/// No configuration is made at this point and defaults are in place.
    pub fn new(i2c: I2C) -> Self{
		let address: u8 = 0x1C;
        let at42qt2120 = At42qt2120 {
            i2c: i2c,
            address: address,
        };
        at42qt2120
    }
	
	/// Setup the device, currently just checks the chip ID. 
	// If this returns OK, you can be pretty sure the AT42QT2120 is alive
	pub fn setup(&mut self) -> Result<(), Error<E>> {
        if (self.read_register(Register::CHIP_ID)?) != 0x3E {
          return Err(Error::WrongDeviceID);
		}
		Ok(())
	}
	
	// Calibrate the device, needed in case keys are stuck or other issues.
	pub fn calibrate(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::CALIBRATE, 0x01)?;
		Ok(())
    }
    
	/// Checks if calibration is running.
	pub fn calibration_running(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register(Register::STATUS)?;
        if status & (1 << 7) == (1 << 7) {
          Ok(true)
        }
        else {
          Ok(false)
        }
	}
	
	//todo 
	pub fn set_detection_integrator(&mut self, value: u8) -> Result<(), Error<E>> {
        self.write_register(Register::DETECTION_INTEGRATOR, value)?;
		Ok(())
    }
	
	/// Setup the slider settings, slider is false for slider, true for wheel.
	pub fn setup_slider(&mut self, slider: bool, enabled: bool) -> Result<(), Error<E>> {
        let mut settings: u8 = 0;
        if slider {
            settings += 1 << 6;
        }
        if enabled {
            settings += 1 << 7;
        }
        self.write_register(Register::SLIDER_OPTIONS, settings)?;
		Ok(())
	}
	
	/// Setup a key. Keys can be enabled or disabled, the lower the threshold, the more sensitive the key.
	/// Settings here also count for the slider/wheel.
	pub fn setup_key(&mut self, key: u8, threshold: u8, enabled: bool) -> Result<(), Error<E>> {
        let mut command : [u8;2] = [0;2];
        
        if threshold == 0 {
            return Err(Error::InvalidThreshold);
        }
        else {
            command[0] = Register::KEY_THRESHOLD_START as u8 + key;
            command[1] = threshold;
            self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        }

        let mut settings: u8 = 0;
        if enabled {
            settings += 1 << 0;
        }
        
	    command[0] = Register::KEY_CONTROL_START as u8 + key;
	    command[1] = settings;
        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        
		Ok(())
	}
	
	/// Checks if any keys are pressed.
	pub fn keys_pressed(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register(Register::STATUS)?;
        if status & (1 << 0) == (1 << 0) {
          Ok(true)
        }
        else {
          Ok(false)
        }
	}
	
	/// Checks if the slider is pressed. 
	// As the slider is made up from 3 keys, the keys_pressed function also returns true when the slider is pressed.
	pub fn slider_pressed(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_register(Register::STATUS)?;
        if status & (1 << 1) == (1 << 1) {
          Ok(true)
        }
        else {
          Ok(false)
        }
	}
	
	/// read the raw detection status register
	pub fn read_status(&mut self) -> Result<u8, Error<E>> {
		self.read_register(Register::STATUS)
	}
	
	/// read the slider position. Only valid when the slider is pressed.
	pub fn read_slider(&mut self,) -> Result<u8, Error<E>> {
		self.read_register(Register::SLIDER_POSITION)
	}
	
	/// read the raw 16 bit key signal
	pub fn read_key_value(&mut self, key: u8,) -> Result<u16, E> {
		let command : [u8;1] = [((Register::KEY_SIGNAL_START as u8) + key*2) ;1];
	    let mut buffer : [u8;1] = [0;1];
		
        self.i2c.write_read(self.address, &command, &mut buffer)?;
        let mut result:u16 = (buffer[0] as u16) << 8;
        
        let command : [u8;1] = [((Register::KEY_SIGNAL_START as u8) + key*2) + 1 ;1];
        self.i2c.write_read(self.address, &command, &mut buffer)?;
        result += buffer[0] as u16;
        
		Ok(result)
	}
	
	/// read all key values. bit 0 is for key 0, bit 1 for key 1 and so forth
	pub fn read_keys(&mut self,) -> Result<u16, Error<E>> {
        let mut result:u16 = (self.read_register(Register::KEY_STATUS2)? as u16) << 8;
        result += self.read_register(Register::KEY_STATUS1)? as u16;
		Ok(result)
	}
	
	/// read if a key is pressed
	pub fn read_key(&mut self, key: u8,) -> Result<bool, Error<E>> {
      let result = self.read_keys()?;
      Ok(0 != (result & (1 << key)))
	}
	
	fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
		let command : [u8;1] = [register as u8;1];
	    let mut buffer : [u8;1] = [0;1];
		
        self.i2c.write_read(self.address, &command, &mut buffer).map_err(Error::I2c)?;
        let result =  buffer[0];
		Ok(result)
	}
	
	fn write_register(&mut self, register: Register, data: u8) -> Result<(), Error<E>> {
		let mut command : [u8;2] = [0;2];
	    command[0] = register as u8;
	    command[1] = data;
		
        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
		Ok(())
	}
}
