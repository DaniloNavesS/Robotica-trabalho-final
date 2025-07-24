import sim
import time
import math  # para funções trigonométricas
import csv  # para salvar os pontos

# Conectar ao CoppeliaSim
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1:
    raise Exception("Falha na conexão com o CoppeliaSim.")

print('Conectado ao servidor remoto do CoppeliaSim')

sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

# Obter handles
res, targetHandle = sim.simxGetObjectHandle(clientID, 'UR3_target', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
    raise Exception("Handle para UR3_target não encontrado.")
res, baseHandle = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
    baseHandle = -1  # sistema de coordenadas global

# Gerar pontos do círculo
r = 0.15             # raio do círculo (em metros)
z = 0.4              # altura fixa
cx = -0.44714        # centro x do círculo (ajuste conforme sua simulação)
cy = 0.0             # centro y do círculo
n = 36               # número de pontos ao longo do círculo (10° entre eles)

circle_points = []
for i in range(n + 1):  # +1 para voltar ao ponto inicial
    angle = 2 * math.pi * i / n
    x = cx
    y = cy + r * math.cos(angle)
    z_pos = z + r * math.sin(angle)
    circle_points.append([x, y, z_pos])

# Mover o dummy ao longo do círculo
for pos in circle_points:
    sim.simxSetObjectPosition(clientID, targetHandle, baseHandle, pos, sim.simx_opmode_oneshot)
    time.sleep(0.2)

# Encerrar
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
sim.simxFinish(clientID)
print('Movimento circular concluído, simulação finalizada')

# Salvar pontos em um arquivo CSV (após a simulação)
with open('trajetoria_circulo.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y', 'z'])
    writer.writerows(circle_points)
print('Trajetória circular salva em trajetoria_circulo.csv')
