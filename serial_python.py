import serial
import matplotlib.pyplot as plt

plt.ion()
ser = serial.Serial('/dev/ttyUSB0', 9600)
counter = 0
data_list = []
for i in range(6):
    data_list.append([])
fig1 = plt.figure()

labels = ['uL', 'LDC', 'velL', 'uR', 'RDC', 'velR']

while 1:
    fun = ser.readline()
    counter += 1
    for i, word in enumerate(fun.split()):
        try:
            print i, word
            value = float(word)
            data_list[i].append(word)
        except ValueError:
            print "null value???", word
    if counter % 5 == 1:
        fig1.clear()
        for i, datas in enumerate(data_list):
            if i == 1 or i == 4:
                plt.subplot(313)
                plt.plot(datas[-200:], label=labels[i])
                plt.legend()
            elif i == 2 or i == 5:
                plt.subplot(312)
                plt.plot(datas[-200:], label=labels[i])
                plt.legend()
            elif i == 0 or i == 3:
                plt.subplot(311)
                plt.plot(datas[-200:], label=labels[i])
                plt.legend()
        plt.draw()


#except KeyboardInterrupt:
    #plt.figure()
    #for datas in data_list:
        #plt.plot(datas)
    #plt.show()


