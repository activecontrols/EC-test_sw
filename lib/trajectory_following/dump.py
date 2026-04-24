import serial
import time

# --- Configuration ---
SERIAL_PORT = 'COM15'  # Change to /dev/ttyUSB0 or similar on Linux/Mac
BAUD_RATE = 57600    # Ensure this matches your microcontroller settings
PAGE_SIZE = 256       # Adjust to match your PAGE_SIZE constant
OUTPUT_FILE = "flash_dump.bin"

def main():
    try:
        # Initialize serial connection
        # timeout=1 ensures we don't hang forever if the MCU stops responding
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
        time.sleep(2) # Wait for MCU to reset after connection
        
        print(f"Connected to {SERIAL_PORT}. Starting dump...")

        # ser.write(b'dump_flash\n') # dump must be triggered from CommsSerial not necessarily usb
        
        with open(OUTPUT_FILE, "wb") as f:
            total_bytes = 0
            

            while True: # failsafe for now in case flag parsing fails for some reason
                # 1. Send 'c' to request the next page
                ser.write(b'c')
                
                # 2. Read the full page
                # Your MCU does: Serial.write(last_page, sizeof(last_page));
                page_data = ser.read(PAGE_SIZE)
                
                if len(page_data) < PAGE_SIZE:
                    print("\nWarning: Timeout or incomplete page received.")
                    break

                cs_A_transmission = ser.read(1)[0]
                cs_B_transmission = ser.read(1)[0]

                cs_A = 0
                cs_B = 0
                for x in page_data:
                    cs_A += x
                    cs_A %= 256
                    cs_B += cs_A
                    cs_B %= 256

                if cs_A_transmission != cs_A or cs_B_transmission != cs_B:
                    print("Page Checksum Failed!")
                    print(f"{cs_A} : {cs_A_transmission}")
                    print(f"{cs_B} : {cs_B_transmission}")
                
                # Save data to file
                f.write(page_data[0:PAGE_SIZE])
                total_bytes += len(page_data)
                
                # 3. Handle the 'c'/'k' handshake loop
                # Your MCU sends 'c' after processing entries or 'k' if 0xFF is hit
                # We need to consume these status bytes from the buffer
                stop_detected = False
                while True:
                    # Read 1 byte to check for status
                    status = ser.read(1)
                    
                    if status == b'k':
                        print("\nEnd of log reached (received 'k').")
                        stop_detected = True
                        break
                    elif status == b'c':
                        # MCU is ready for the next page request
                        break
                    elif status == b'':
                        # Timeout
                        break
                
                print(f"\rDownloaded {total_bytes} bytes...", end="")
                
                if stop_detected:
                    break

        print(f"\nSuccess! Data saved to {OUTPUT_FILE}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nDump interrupted by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()