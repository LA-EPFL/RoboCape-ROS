import pyUblox.ublox

def main():
    gps = pyUblox.ublox.Ublox(port="/dev/ttyO4", baudrate=9600, timeout=0)

    print(gps.receive_message())

if __name__ == '__main__':
    main()
