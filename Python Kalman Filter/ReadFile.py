class getData:
    def __init__(self):
        self.mag_0 = []
        self.mag_1 = []
        self.acc_0 = []
        self.acc_1 = []
        self.gyro = []
        self.timestamp = []
        self.quart_wahba = []
        self.quart_xk = []
        self.quart_gyro = []
        self.readFile()

    @staticmethod
    def getArray(line, n):
        arr = []
        string = line.split(':')[1]
        string_list = string.split(',')
        for val in string_list:
            arr.append(float(val))
        return arr

    def readFile(self):
        with open('D:/GITProjects/Kalman Filtering Server/PoseEstimationKF/Sensor_CSV/KalmanFilter.txt') as file:
            lines = file.readlines()
            count = 0
            for line in lines:
                if('mag_0' in line):
                    self.mag_0 = getData.getArray(line,3) 
                elif('acc_0' in line):
                    self.acc_0 = getData.getArray(line,3) 
                elif('Acc_1' in line):
                    self.acc_1.append(getData.getArray(line,3))
                elif('Mag_1' in line):
                    self.mag_1.append(getData.getArray(line,3))
                elif('q_gyro' in line):
                    self.quart_gyro.append(getData.getArray(line,4))
                elif('gyro' in line):
                    self.gyro.append(getData.getArray(line,3))
                elif('T' in line):
                    self.timestamp.append(getData.getArray(line,1))
                elif('Wahba_quart' in line):
                    self.quart_wahba.append(getData.getArray(line,4))
                elif('X_k' in line):
                    self.quart_xk.append(getData.getArray(line,4))