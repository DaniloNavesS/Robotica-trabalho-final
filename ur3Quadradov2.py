import sim              # biblioteca de API remota do CoppeliaSim (sim.py deve estar no path)
import time

# Conectar ao CoppeliaSim (certifique-se de que a simulação já esteja rodando)
sim.simxFinish(-1)  # fecha possíveis conexões anteriores
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1:
    raise Exception("Falha na conexão com o CoppeliaSim. Verifique se a simulação está rodando e o script remoteApi foi iniciado.")

print('Conectado ao servidor remoto do CoppeliaSim')

# Iniciar simulação (modo bloqueante)
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

# Obter handles dos objetos: dummy alvo e, opcionalmente, base do UR3
res, targetHandle = sim.simxGetObjectHandle(clientID, 'UR3_target', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
    raise Exception("Handle para UR3_target não encontrado. Verifique o nome do dummy no CoppeliaSim.")
res, baseHandle = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
    baseHandle = -1  # usar sistema de coordenadas mundial

# Definir coordenadas dos vértices do quadrado RELATIVOS à base do UR3 (z=0 é o topo da mesa)
square_points = [
    [-0.44714,  +0.18652, 0.34762],   # ponto 1
    [-0.44714,  +0.18652, 0.5],       # ponto 2
    [-0.44714,  -0.18652, 0.5],       # ponto 3
    [-0.44714,  -0.18652, 0.34762],   # ponto 4
    [-0.44714,  +0.18652, 0.34762]    # volta ao ponto inicial
]

# Mover o dummy de meta para cada ponto do quadrado, relativo à base do robô
for pos in square_points:
    sim.simxSetObjectPosition(clientID, targetHandle, baseHandle, pos, sim.simx_opmode_oneshot)
    time.sleep(1)  # aguardar movimento (ajuste conforme necessário)

# Parar simulação e desconectar
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
sim.simxFinish(clientID)
print('Movimento concluído, simulação finalizada')
