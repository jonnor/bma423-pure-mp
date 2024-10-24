
from machine import SoftI2C, Pin
import time
import array
import bma423


# decode an entire block of acceleration values
def decode_acceleration(sensor, inp, out):
    # inp is the raw bytes for a set of acceleration samples, typically read from FIFO
    # NOTE: must be without FIFO headers!
    
    bytes_per_sample = 2
    axes = 3
    n_samples = len(inp) // (bytes_per_sample*axes)
    for sample_no in range(0, n_samples):
        sample_start = sample_no*bytes_per_sample*axes
        rawdata = memoryview(inp)[sample_start:]
        acc_x = (rawdata[0] >> 4) | (rawdata[1] << 4)
        acc_y = (rawdata[2] >> 4) | (rawdata[3] << 4)
        acc_z = (rawdata[4] >> 4) | (rawdata[5] << 4)
        acc_x = sensor.convert_to_int12(acc_x)
        acc_y = sensor.convert_to_int12(acc_y)
        acc_z = sensor.convert_to_int12(acc_z)
        acc_x = sensor.normalize_reading(acc_x)
        acc_y = sensor.normalize_reading(acc_y)
        acc_z = sensor.normalize_reading(acc_z)
        out[(sample_no*axes)+0] = acc_x
        out[(sample_no*axes)+1] = acc_y
        out[(sample_no*axes)+2] = acc_z

def main():

    accel_samples = 100 # how many samples to wait before reading. BMA423 has 1024 bytes FIFO, enough for 150+ samples
    samplerate = 100

    # pre-allocate buffers
    # raw data (bytes). n_samples X 3 axes X 2 bytes
    accel_buffer = bytearray(accel_samples*3*2)
    # normalized output data (in g). n_samples X 3 axes
    accel_array = array.array('f', list(range(0, accel_samples*3)))

    # setup sensor
    i2c = SoftI2C(scl=11,sda=10)
    sensor = bma423.BMA423(i2c)
    sensor.fifo_enable()
    sensor.set_accelerometer_freq(samplerate)
    sensor.fifo_clear() # discard any samples lying around in FIFO
    time.sleep_ms(100)

    while True:
        
        # wait until we have enough samples
        fifo_level = sensor.fifo_level()
        if fifo_level >= len(accel_buffer):
            
            # read data
            read_start = time.ticks_ms()
            sensor.fifo_read(accel_buffer)
            read_dur = time.ticks_diff(time.ticks_ms(), read_start)
            
            # decode it
            decode_start = time.ticks_ms()
            decode_acceleration(sensor, accel_buffer, accel_array)
            decode_dur = time.ticks_diff(time.ticks_ms(), read_start)

            print('fifo-read', read_start/1000, fifo_level, read_dur, decode_dur)

            # print all samples
            for n in range(0, accel_samples):
                sample_start = n * 3
                x,y,z = accel_array[sample_start:sample_start+3]
                out = '{0:.2f},{1:.2f},{2:.2f}'.format(x, y, z)
                print(n, out)
        
        # limit how often we check
        time.sleep_ms(10)

if __name__ == '__main__':
    main()