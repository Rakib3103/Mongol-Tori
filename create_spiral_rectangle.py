import serial
import pynmea2
from geopy.distance import distance
import matplotlib.pyplot as plt

# Define the serial port and baud rate for your GPS module
serial_port = 'COM10'  # Replace 'COM3' with the appropriate port name
baud_rate = 9600

# Define the starting dimensions and growth rate of the spiral
starting_side_length = 1  # in meters
growth_rate = 2  # in meters
num_coordinates = 30

# Create a serial connection
ser = serial.Serial(serial_port, baud_rate)

# Read data from the serial port and save the coordinates
coordinates = []

while len(coordinates) < num_coordinates:
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

            # Calculate the coordinates of the spiral
            center_point = (current_latitude, current_longitude)
            spiral_side_length = starting_side_length + (len(coordinates) // 4) * growth_rate
            spiral_angle = (len(coordinates) % 4) * 90

            spiral_point = distance(meters=spiral_side_length).destination(center_point, spiral_angle)

            print(f'Spiral Point: {spiral_point[0]}, {spiral_point[1]}')

            # Store the coordinates
            coordinates.append((spiral_point[0], spiral_point[1]))

    except serial.SerialException as e:
        print(f'An error occurred: {e}')
        break
    except pynmea2.ParseError as e:
        print(f'Error parsing NMEA data: {e}')

# Extract latitude and longitude values from the coordinates
latitude_values = [coordinate[0] for coordinate in coordinates]
longitude_values = [coordinate[1] for coordinate in coordinates]

# Plot the spiral coordinates
plt.plot(longitude_values, latitude_values, color='blue', marker='o')

# Set plot title and labels
plt.title('Spiral Coordinates')
plt.xlabel('Longitude')
plt.ylabel('Latitude')

# Show the plot
plt.show()
