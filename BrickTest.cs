
using System;


public class BrickTest
{
	//int result = brick.SetTimeout();
	//Console.WriteLine("Result: {0}", result);

	//~ for (int i = 0; i < 10; i++)
	//~ {
		//~ brick.Transmit(1, new byte[] { 5, 200, 0, 0, 0 });
		//~ byte[] data;
		//~ int result = brick.Receive(1000, out data);
		//~ Console.Write("Result ({0}): ", result);
		//~ if (data != null)
			//~ foreach (byte b in data)
				//~ Console.Write("{0}, ", b);
		//~ Console.WriteLine();
	//~ }


	static void SensorTest()
	{
		BrickPi brick = new BrickPi();
		
		brick.SensorType[BrickPi.PORT_1] = BrickPi.TYPE_SENSOR_TOUCH;
		brick.SetupSensors();
		while (true)
		{	
			brick.UpdateValues();
			Console.WriteLine("Sensor 1: {0}", brick.Sensor[BrickPi.PORT_1]);
			System.Threading.Thread.Sleep(500);
		}
	}

	static void MotorTest()
	{
		BrickPi brick = new BrickPi();
		
		brick.Timeout = 200;
		brick.MotorEnable[BrickPi.PORT_A] = 1;
		brick.MotorEnable[BrickPi.PORT_B] = 1;
		brick.MotorEnable[BrickPi.PORT_C] = 1;
		brick.MotorEnable[BrickPi.PORT_D] = 1;
		brick.SetupSensors();
		brick.SetTimeout();
		
		int power = 255;
		while (true)
		{	
			Console.WriteLine("Forward with {0}", power);
			brick.MotorSpeed[BrickPi.PORT_A] = power;
			brick.MotorSpeed[BrickPi.PORT_B] = power;
			brick.MotorSpeed[BrickPi.PORT_C] = power;
			brick.MotorSpeed[BrickPi.PORT_D] = power;
			
			long start = DateTime.Now.Ticks;
			while (DateTime.Now.Ticks - start < 3 * 10000000L)
			{
				Console.WriteLine("Encoder: {0}, {1}, {2}, {3}", brick.Encoder[BrickPi.PORT_A], brick.Encoder[BrickPi.PORT_B], brick.Encoder[BrickPi.PORT_C], brick.Encoder[BrickPi.PORT_D]);
				brick.UpdateValues();
				System.Threading.Thread.Sleep(100);
			}
			
			Console.WriteLine("Backward with {0}", power);
			brick.MotorSpeed[BrickPi.PORT_A] = -power;
			brick.MotorSpeed[BrickPi.PORT_B] = -power;
			brick.MotorSpeed[BrickPi.PORT_C] = -power;
			brick.MotorSpeed[BrickPi.PORT_D] = -power;
			
			start = DateTime.Now.Ticks;
			while (DateTime.Now.Ticks - start < 3 * 10000000L)
			{
				Console.WriteLine("Encoder: {0}, {1}, {2}, {3}", brick.Encoder[BrickPi.PORT_A], brick.Encoder[BrickPi.PORT_B], brick.Encoder[BrickPi.PORT_C], brick.Encoder[BrickPi.PORT_D]);
				brick.UpdateValues();
				System.Threading.Thread.Sleep(100);
			}
			
			power += 10;
			if (power > 255)
				power = 255;
		}
	}
	
    public static void Main()
    {
		MotorTest();
    }
}
