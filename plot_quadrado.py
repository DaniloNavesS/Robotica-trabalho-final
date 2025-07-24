import matplotlib.pyplot as plt
import csv

x_out = []
y_out = []
z_out = []

with open('trajetoria_ur3v2.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x_out.append(float(row['x_out']))
        y_out.append(float(row['y_out']))
        z_out.append(float(row['z_out']))

# Pontos de referência (os mesmos do arquivo)
pontos = list(zip(x_out, y_out, z_out))
cores = ['magenta', 'yellow', 'cyan', 'red']  # Ponto 1 (preto) removido
labels = ['Ponto 2', 'Ponto 3', 'Ponto 4', 'Ponto 5']

plt.figure(figsize=(10, 7))
plt.plot(y_out, z_out, 'b-', linewidth=4, label='Trajetória UR3')

# Plota os pontos 2 em diante
for (x, y, z), cor, label in zip(pontos[1:], cores, labels):
    plt.plot(y, z, 'o', color=cor, markersize=10, label=label)

# Plota o início/ponto 1 em verde
plt.plot(y_out[0], z_out[0], 'o', color='green', markersize=10, label='Início/Ponto 1')

plt.xlabel('Posição Y (m)')
plt.ylabel('Posição Z (m)')
plt.title('"Desenho" do UR3 (vista YZ)')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.savefig("trajetoria_ur3v2_yz.png")
plt.show() 