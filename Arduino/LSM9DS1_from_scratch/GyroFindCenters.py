import matplotlib.pyplot as plt

fileName = input("Please enter file name ")
inFile = open(str(fileName + ".txt"), 'r')
gyroData = [[], [], []]
data = [num for num in inFile.readline().split()]
while data != []:
    for i in range(len(data)):
        gyroData[i].append(data[i])
    data =  inFile.readline().split()
inFile.close()

gyroData = [[float(num) for num in gyroData[0]], [float(num) for num in gyroData[1]], [float(num) for num in gyroData[2]]]
for data in gyroData:
    plt.plot(data)

centers = [(max(gyroData[0]) + min(gyroData[0])) / 2, (max(gyroData[1]) + min(gyroData[1])) / 2, (max(gyroData[2]) + min(gyroData[2])) / 2]
print("Centers [x, y, z] =", centers)

for center in centers:
    plt.plot((0, 400), (center, center))
plt.show()
