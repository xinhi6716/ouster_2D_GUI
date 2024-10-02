from SOIC_ouster.SOIC import SOIC_Ouster


if __name__ == '__main__':
    soic_ouster = SOIC_Ouster('./A1.pcap','./A1.json')
    soic_ouster.show(time_stamp=True)
    soic_ouster.save('output.avi')