import serial
import pynmea2
from geopy.distance import distance
import matplotlib.pyplot as plt

# Define the serial port and baud rate for your GPS module
serial_port = 'COM10'  # Replace 'COM3' with the appropriate port name
baud_rate = 9600

# Define the dimensions of the rectangle
side_length = 2  # in meters

# Create a serial connection
ser = serial.Serial(serial_port, baud_rate)

# Read data from the serial port
while True:
    try:
        line = ser.readline()
        try:
            line = line.decode('latin-1').strip()
        except UnicodeDecodeError:
            continue  # Skip decoding errors and continue reading the next line

        if line.startswith('$GNGGA'):
            msg = pynmea2.parse(line)
            current_latitude = msg.latitude
            current_longitude = msg.longitude
            print(f'Current Coordinate: {current_latitude}, {current_longitude}')

            # Calculate the coordinates of the rectangle's corners
            center_point = (current_latitude, current_longitude)
            north_point = distance(meters=side_length / 2).destination(center_point, 0)
            south_point = distance(meters=side_length / 2).destination(center_point, 180)
            east_point = distance(meters=side_length / 2).destination(center_point, 90)
            west_point = distance(meters=side_length / 2).destination(center_point, 270)

            print(f'North Point: {north_point[0]}, {center_point[1]}')
            print(f'South Point: {south_point[0]}, {center_point[1]}')
            print(f'East Point: {center_point[0]}, {east_point[1]}')
            print(f'West Point: {center_point[0]}, {west_point[1]}')

            # Plot the coordinates on a scatter plot
            plt.scatter(current_longitude, current_latitude, color='blue', label='Current Coordinate')
            plt.scatter(center_point[1], north_point[0], color='red', label='North Point')
            plt.scatter(center_point[1], south_point[0], color='green', label='South Point')
            plt.scatter(east_point[1], center_point[0], color='orange', label='East Point')
            plt.scatter(west_point[1], center_point[0], color='purple', label='West Point')

            # Set plot title and labels
            plt.title('Rectangle Coordinates')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')

            # Add legend
            plt.legend()

            # Show the plot
            plt.show()

            # Break out of the loop after plotting once
            break

    except serial.SerialException as e:
        print(f'An error occurred: {e}')
        break
    except pynmea2.ParseError as e:
        print(f'Error parsing NMEA data: {e}')
