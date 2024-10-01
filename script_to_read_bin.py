import struct

# Open the .bin file in binary mode
with open('C:\\Users\\fabate\\Downloads\\hhhh.bin', 'rb') as bin_file:
    # Read the entire file into a byte array
    data = bin_file.read()
    
    # Example: Assuming we know the structure of the data in memory
    # Let's say the first part of the data contains an integer (4 bytes)
    # Followed by a float (4 bytes) and a string of 10 characters (10 bytes)
print(len(data))    
for i in range(0, len(data), 4):
    # Extract the next 4-byte chunk
    chunk = data[i:i+4]
    
    # Ensure we have exactly 4 bytes to unpack
    if len(chunk) == 4:
        # Unpack the 4 bytes as a little-endian float
        float_value = struct.unpack('<f', chunk)[0]
        
        # Print the float value and raw byte data
        print(f"Float value at offset {i}: {float_value}")
        print(f"Raw data at offset {i}: {chunk}")
    else:
        print(f"Insufficient data at offset {i}, skipping.")

    
