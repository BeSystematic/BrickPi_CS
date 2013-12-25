using System;
using System.Collections.Generic;
using System.IO.Ports;

public class BrickPi
{
	SerialPort _serialPort;
	
	public const int PORT_A = 0;
	public const int PORT_B = 1;
	public const int PORT_C = 2;
	public const int PORT_D = 3;

	public const int PORT_1 = 0;
	public const int PORT_2 = 1;
	public const int PORT_3 = 2;
	public const int PORT_4 = 3;

	public const int MASK_D0_M = 0x01;
	public const int MASK_D1_M = 0x02;
	public const int MASK_9V = 0x04;
	public const int MASK_D0_S = 0x08;
	public const int MASK_D1_S = 0x10;

	public const int BYTE_MSG_TYPE 	      = 0; // MSG_TYPE is the first byte.
	public const int MSG_TYPE_CHANGE_ADDR = 1; // Change the UART address.
	public const int MSG_TYPE_SENSOR_TYPE = 2; // Change/set the sensor type.
	public const int MSG_TYPE_VALUES      = 3; // Set the motor speed and direction, and return the sesnors and encoders.
	public const int MSG_TYPE_E_STOP      = 4; // Float motors immidately
	public const int MSG_TYPE_TIMEOUT_SETTINGS = 5; // Set the timeout
// New UART address (MSG_TYPE_CHANGE_ADDR)e
	public const int BYTE_NEW_ADDRESS = 1;

// Sensor setup (MSG_TYPE_SENSOR_TYPE)
	public const int BYTE_SENSOR_1_TYPE = 1;
	public const int BYTE_SENSOR_2_TYPE = 2;

	public const int BYTE_TIMEOUT=1;

	public const int TYPE_MOTOR_PWM = 0;
	public const int TYPE_MOTOR_SPEED = 1;
	public const int TYPE_MOTOR_POSITION = 2;

	public const int TYPE_SENSOR_RAW = 0; // - 31
	public const int TYPE_SENSOR_LIGHT_OFF = 0;
	public const int TYPE_SENSOR_LIGHT_ON = (MASK_D0_M | MASK_D0_S);
	public const int TYPE_SENSOR_TOUCH = 32;
	public const int TYPE_SENSOR_ULTRASONIC_CONT = 33;
	public const int TYPE_SENSOR_ULTRASONIC_SS = 34;
	public const int TYPE_SENSOR_RCX_LIGHT = 35; // tested minimally
	public const int TYPE_SENSOR_COLOR_FULL = 36;
	public const int TYPE_SENSOR_COLOR_RED = 37;
	public const int TYPE_SENSOR_COLOR_GREEN = 38;
	public const int TYPE_SENSOR_COLOR_BLUE = 39;
	public const int TYPE_SENSOR_COLOR_NONE = 40;
	public const int TYPE_SENSOR_I2C = 41;
	public const int TYPE_SENSOR_I2C_9V = 42;

	public const int BIT_I2C_MID = 0x01; // Do one of those funny clock pulses between writing and reading. defined for each device.
	public const int BIT_I2C_SAME = 0x02; // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

	public const int INDEX_RED = 0;
	public const int INDEX_GREEN = 1;
	public const int INDEX_BLUE = 2;
	public const int INDEX_BLANK = 3;

	private int[] Address = new int [] { 1, 2 };
	
	public int[] MotorSpeed = new int [4];
    public int[] MotorEnable  = new int [4];
        
    public int[] EncoderOffset = new int [4];
    public int[] Encoder  = new int [4];

    public int[] Sensor  = new int [4];
    public int[,] SensorArray = new int [ 4, 4 ];
    public int[] SensorType = new int [4];
    public int[,] SensorSettings = new int [ 4, 8 ];

    public int[] SensorI2CDevices  = new int [4];
    public int[] SensorI2CSpeed  = new int [4];
    public int[,] SensorI2CAddr =  new int [ 4, 8 ];
    public int[,] SensorI2CWrite =  new int [ 4, 8 ];
    public int[,] SensorI2CRead =  new int [ 4, 8 ];
    public int[,,] SensorI2COut =  new int [ 4, 8, 16 ];
    public int[,,] SensorI2CIn =  new int [ 4, 8, 16 ];
    
    public int Timeout = 0;

	public BrickPi()
	{
		string portName = "/dev/ttyAMA0";
		int baudRate = 500000;
		_serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
		_serialPort.Encoding = new System.Text.UnicodeEncoding();
		
		Open();
	}

	public void Open()
	{
		if (!_serialPort.IsOpen)
		{
			_serialPort.Open();
			if (_serialPort.IsOpen)
				Console.WriteLine("Serial port {0} opened with {1} baud", _serialPort.PortName, _serialPort.BaudRate);
		}
		
		if (Type.GetType("Mono.Runtime") == null) 
			return; //It is not mono === not linux! 
			
        string arg = String.Format("-F {0} speed {1}",_serialPort.PortName , _serialPort.BaudRate);
        var proc = new System.Diagnostics.Process
            {
                EnableRaisingEvents = false,
                StartInfo = {FileName = @"stty", Arguments = arg}
            };
        proc.Start();
        proc.WaitForExit();
	}

	public void Close()
	{	
		_serialPort.Close();
	}
	
	private static string[] GetPortNames()
    {
        int p = (int)Environment.OSVersion.Platform;
        List<string> serial_ports = new List<string>();

        // Are we on Unix?
        if (p == 4 || p == 128 || p == 6)
        {
            string[] ttys = System.IO.Directory.GetFiles("/dev/", "tty*");
            foreach (string dev in ttys)
            {
                if (dev.StartsWith("/dev/ttyS") || dev.StartsWith("/dev/ttyUSB") || dev.StartsWith("/dev/ttyA"))
                {
                    serial_ports.Add(dev);
                }
            }
        }
        else
        {
            serial_ports.AddRange(SerialPort.GetPortNames());
        }

        return serial_ports.ToArray();
    }
	
	public void Transmit(int device, byte[] data)
	{		
		byte[] message = new byte[data.Length + 3];
		message[0] = (byte)device;
		message[1] = (byte)(device + data.Length);
		message[2] = (byte)data.Length;
		for (int i = 0; i < data.Length; i++)
		{
			message[1] += data[i];
			message[3 + i] = data[i];
		}
		
		//~ Console.Write("Transmit({0}): ", message.Length);
		//~ foreach (byte b in message)
			//~ Console.Write("{0}, ", (int)b);
		//~ Console.WriteLine();
		
		_serialPort.Write(message, 0, message.Length);
	}
	
	public int Receive(int timeout, out byte[] data)
	{	
		data = null;

		if (!_serialPort.IsOpen)
			return -1;
		_serialPort.ReadTimeout = timeout;
		
//		long start = DateTime.Now.Ticks;
//		while (_serialPort.BytesToRead <= 0)
//		{
//			if (DateTime.Now.Ticks - start >= timeout * 10000)
//				return -2;
//		}
		
		int chksum;
		int length;
		try 
		{
			chksum = _serialPort.ReadByte();
			length = _serialPort.ReadByte();
			data = new byte[length];
			_serialPort.Read(data, 0, length);
		}
		catch (Exception)
		{
			return -2;
		}
		
		//~ Console.Write("Receive({0}): {1}, {2}, ", data.Length + 2, chksum, length);
		//~ foreach (byte b in data)
			//~ Console.Write("{0}, ", (int)b);
		//~ Console.WriteLine();
		

		int chksumchk = length;
		foreach (byte b in data)
			chksumchk += b;
		if ((chksumchk % 256) != chksum)
			return -5;

		return 0;
	}

	public int SetTimeout()
	{
        for (int i = 0; i < 2; i++)
        {
			byte[] Array = new byte[5];
			Array[BYTE_MSG_TYPE] = (byte)MSG_TYPE_TIMEOUT_SETTINGS;
			Array[BYTE_TIMEOUT] = (byte)(Timeout & 0xFF);
			Array[BYTE_TIMEOUT + 1] = (byte)((Timeout / 256 ) & 0xFF);
			Array[BYTE_TIMEOUT + 2] = (byte)((Timeout / 65536 ) & 0xFF);
			Array[BYTE_TIMEOUT + 3] = (byte)((Timeout / 16777216) & 0xFF);
			
			Transmit(Address[i], Array);
			
			byte[] InArray;
			int res = Receive(2500, out InArray);
			
			if (res != 0)
				return res;
			
			if ((InArray.Length != 1) || (InArray[BYTE_MSG_TYPE] != MSG_TYPE_TIMEOUT_SETTINGS))
				return -1;
		}
        return 0;
	}
	
	
	private int GetBits(int byteOffset, int bitOffset, int bits, ref byte[] data, ref int totalBitOffset)
	{
		int Result = 0;
		
		for (int i = bits - 1; i >= 0; i--)
		{
			int offset = (bitOffset + totalBitOffset + i);
			int index = (byteOffset + (offset / 8));
			Result *= 2;
			Result |= ((data[index] >> (offset % 8)) & 0x01);    
		}
		totalBitOffset += bits;
		
		return Result;
	}

	private int BitsNeeded(int value)
	{
		for (int i = 0; i < 32; i++)
		{
			if (value == 0)
				return i;
			value /= 2;
		}
		return 31;
	}

	
	private void AddBits(int byteOffset, int bitOffset, int bits, int value, ref byte[] data, ref int totalBitOffset)
	{
		for (int i = 0; i < bits; i++)
		{
			if ((value & 0x01) != 0)
			{
				int offset = (bitOffset + totalBitOffset + i);
				int index = (byteOffset + offset / 8);
				data[index] |= (byte)(0x01 << (offset % 8));
			}
			value /= 2;
		}
		totalBitOffset += bits;
	}
	
	public int SetupSensors()
	{
		for (int i = 0; i < 2; i++)
		{
			byte[] data = new byte[256];
			int Bit_Offset = 0;
			data[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE;
			data[BYTE_SENSOR_1_TYPE] = (byte)SensorType[PORT_1 + (i * 2)];
			data[BYTE_SENSOR_2_TYPE] = (byte)SensorType[PORT_2 + (i * 2)];
			for (int ii = 0; ii < 2; ii++)
			{
				int port = (i * 2) + ii;
				if (data[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C
					|| data[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C_9V)
				{
					AddBits(3, 0, 8, SensorI2CSpeed[port], ref data, ref Bit_Offset);
					
					if (SensorI2CDevices[port] > 8)
						SensorI2CDevices[port] = 8;
					
					if (SensorI2CDevices[port] == 0)
						SensorI2CDevices[port] = 1;
					
					AddBits(3, 0, 3, (SensorI2CDevices[port] - 1), ref data, ref Bit_Offset);
					
					for (int device = 0; device < SensorI2CDevices[port]; device++)
					{
						AddBits(3, 0, 7, (SensorI2CAddr[port, device] >> 1), ref data, ref Bit_Offset);
						AddBits(3, 0, 2, SensorSettings[port, device], ref data, ref Bit_Offset);
						if ((SensorSettings[port, device] & BIT_I2C_SAME) != 0)
						{          
							AddBits(3, 0, 4, SensorI2CWrite[port, device], ref data, ref Bit_Offset);
							AddBits(3, 0, 4, SensorI2CRead [port, device], ref data, ref Bit_Offset);
							for (int out_byte = 0; out_byte < SensorI2CWrite[port, device]; out_byte++)
							{
								AddBits(3, 0, 8, SensorI2COut[port, device, out_byte], ref data, ref Bit_Offset);
							}
						}
					}
				}
			}
			int UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 3);
			System.Array.Resize<byte>(ref data, UART_TX_BYTES);
			Transmit(Address[i], data);
			
			byte[] InArray;
			int res = Receive(2500, out InArray);
			
			if (res != 0)
				return res;
			
			if (!(InArray.Length == 1 && data[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
				return -1;
		}
		return 0;
	}



	private int RequestValues(int i, out byte[] data)
	{
		data = new byte[256];
		data[BYTE_MSG_TYPE] = MSG_TYPE_VALUES;
		for (int retry = 0; retry < 2; retry++) 
		{
			int Bit_Offset = 0;
			
			for (int ii = 0; ii < 2; ii++)
			{
				int port = (i * 2) + ii;
				if (EncoderOffset[port] != 0)
				{
					int Temp_Value = EncoderOffset[port];
					int Temp_ENC_DIR = 0;

					AddBits(1, 0, 1, 1, ref data, ref Bit_Offset);
					if (Temp_Value < 0)
					{
					  Temp_ENC_DIR = 1;
					  Temp_Value *= (-1);
					}
					int Temp_BitsNeeded = (BitsNeeded(Temp_Value) + 1);
					AddBits(1, 0, 5, Temp_BitsNeeded, ref data, ref Bit_Offset);
					Temp_Value *= 2;
					Temp_Value |= Temp_ENC_DIR;
					AddBits(1, 0, Temp_BitsNeeded, Temp_Value, ref data, ref Bit_Offset);
				}
				else
				{
					AddBits(1, 0, 1, 0, ref data, ref Bit_Offset);
				}
			}
			
			for (int ii = 0; ii < 2; ii++)
			{
				int port = (i * 2) + ii;
				int speed = MotorSpeed[port];
				int dir = 0;
				if (speed < 0)
				{
					dir = 1;
					speed *= (-1);
				}
				if (speed > 255)
				{
					speed = 255;
				}
				int motorBits = ((((speed & 0xFF) << 2) | (dir << 1) | (MotorEnable[port] & 0x01)) & 0x3FF);
				Console.WriteLine("MotorBits: 0x{0:x10}", motorBits);
				AddBits(1, 0, 10, motorBits, ref data, ref Bit_Offset);
			}
			
			for (int ii = 0; ii < 2; ii++)
			{
				int port = (i * 2) + ii;
				if( SensorType[port] == TYPE_SENSOR_I2C
					|| SensorType[port] == TYPE_SENSOR_I2C_9V)
				{
					for (int device = 0; device < SensorI2CDevices[port]; device++)
					{
						if ((SensorSettings[port,device] & BIT_I2C_SAME) == 0)
						{
							AddBits(1, 0, 4, SensorI2CWrite[port, device], ref data, ref Bit_Offset);
							AddBits(1, 0, 4, SensorI2CRead [port, device], ref data, ref Bit_Offset);
							for (int out_byte = 0; out_byte < SensorI2CWrite[port, device]; out_byte++)
							{
								AddBits(1, 0, 8, SensorI2COut[port, device, out_byte], ref data, ref Bit_Offset);
							}
						}
					}
				}
			}
			
			int UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 1);
			System.Array.Resize<byte>(ref data, UART_TX_BYTES);
			Transmit(Address[i], data);
			
			int result = Receive(7500, out data);
			
			if (result != -2)
			{                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
				EncoderOffset[((i * 2) + PORT_A)] = 0;
				EncoderOffset[((i * 2) + PORT_B)] = 0;
			}
			
			if (result != 0)
				Console.WriteLine("BrickPi Error: {0}", result);
			
			if ((result == 0) && (data[BYTE_MSG_TYPE] == MSG_TYPE_VALUES))
				return 0;
		}
		Console.WriteLine("Retry failed.");
		return -1;
	}		
	
	private int UpdateValues(int i, byte[] data)
	{
		int Bit_Offset = 0;
		
		int[] Temp_BitsUsed = new int[] {0, 0};         // Used for encoder values
		Temp_BitsUsed[0] = GetBits(1, 0, 5, ref data, ref Bit_Offset);
		Temp_BitsUsed[1] = GetBits(1, 0, 5, ref data, ref Bit_Offset);
		int Temp_EncoderVal;
		
		for (int ii = 0; ii < 2; ii++)
		{
			Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii], ref data, ref Bit_Offset);
			if ((Temp_EncoderVal & 0x01) != 0)
			{
				Temp_EncoderVal /= 2;
				Encoder[ii + (i * 2)] = Temp_EncoderVal * (-1);
			}
			else
			{
				Encoder[ii + (i * 2)] = (Temp_EncoderVal / 2);
			}      
		}

		for (int ii = 0; ii < 2; ii++)
		{
			int port = ii + (i * 2);
			switch (SensorType[port])
			{
				case TYPE_SENSOR_TOUCH:
					Sensor[port] = GetBits(1, 0, 1, ref data, ref Bit_Offset);
					break;
				case TYPE_SENSOR_ULTRASONIC_CONT:
				case TYPE_SENSOR_ULTRASONIC_SS:
					Sensor[port] = GetBits(1, 0, 8, ref data, ref Bit_Offset);
					break;
				case TYPE_SENSOR_COLOR_FULL:
					Sensor[port] = GetBits(1, 0, 3, ref data, ref Bit_Offset);
					SensorArray[port, INDEX_BLANK] = GetBits(1, 0, 10, ref data, ref Bit_Offset);
					SensorArray[port, INDEX_RED  ] = GetBits(1, 0, 10, ref data, ref Bit_Offset);                
					SensorArray[port, INDEX_GREEN] = GetBits(1, 0, 10, ref data, ref Bit_Offset);
					SensorArray[port, INDEX_BLUE ] = GetBits(1, 0, 10, ref data, ref Bit_Offset);
					break;          
				case TYPE_SENSOR_I2C:
				case TYPE_SENSOR_I2C_9V:
					Sensor[port] = GetBits(1, 0, SensorI2CDevices[port], ref data, ref Bit_Offset);
			  
					for (int device = 0; device < SensorI2CDevices[port]; device++)
					{
						if ((Sensor[port] & (0x01 << device)) != 0)
						{
							for (int in_byte = 0; in_byte < SensorI2CRead[port, device]; in_byte++)
							{
								SensorI2CIn[port, device, in_byte] = GetBits(1, 0, 8, ref data, ref Bit_Offset);
							}
						}
					}	
					break;      
				case TYPE_SENSOR_LIGHT_OFF:
				case TYPE_SENSOR_LIGHT_ON:
				case TYPE_SENSOR_RCX_LIGHT:
				case TYPE_SENSOR_COLOR_RED:
				case TYPE_SENSOR_COLOR_GREEN:
				case TYPE_SENSOR_COLOR_BLUE:
				case TYPE_SENSOR_COLOR_NONE:
				default:
					Sensor[(ii + (i * 2))] = GetBits(1, 0, 10, ref data, ref Bit_Offset);
					break;
			} 
		}
		return 0;
	}
	
	public int UpdateValues()
	{
		for (int i = 0; i < 2; i++)
		{
			byte[] data;
			int result = RequestValues(i, out data);

			if (result != 0)
				return result;
				
			UpdateValues(i, data);
		}
		return 0;
	}
};
