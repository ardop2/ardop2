import time
import serial

class SerialPacket:

    def __init__(self, port, baud):

        self.ser = serial.Serial(port, baud, timeout=1)
        print '\nSerial device connncted at %s'%(self.ser.port)

        self.data = []
        self.packet = []
        self.checksumEnable = False
        self.packetHead = [chr(200)]
        self.packetTail = [chr(201)]


    def close(self):

        if self.ser.isOpen(): self.ser.close()
        print '\nSerial device disconnected'
        

    def getChecksum(self, packet):
        
        checksum = 0
        for i in packet: checksum ^= ord(i)
        return chr(checksum)
    

    def generatePacket(self):

        self.packet = self.packetHead
        for i in self.data:
            if type(i) is int: self.packet.append(chr(i))
            elif type(i) is str:
                for j in i: self.packet.append(j)
            else: print '\nError generating packet'
        
        if self.checksumEnable: self.packet += [self.getChecksum(self.packet[1:])]
        self.packet += self.packetTail
        
    
    def sendPacket(self):

        self.generatePacket()
        print '\nData:', self.data
        print 'Packet:', [i.encode('hex') for i in self.packet], '\n'
        
        for i in self.packet:
            print 'Sending Byte:', ord(i)
            self.ser.write(i)
            time.sleep(0.2)


    def echo(self):

        while self.ser.in_waiting:
            print self.ser.readlines()
        

    def getPacket(self):
        
        pass


if __name__ == '__main__' :

    print 'UNIT TEST:'
    
    try:
        dev = SerialPacket('Hi', 9600)
        dev.checksumEnable = False
        dev.data = [60, 30, 90]
        dev.sendPacket()

    except Exception as err:
        print err

    dev.close()
