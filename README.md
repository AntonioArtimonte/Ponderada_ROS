# Ponderada ROS2 da Semana 1 do módulo 06 de E.C

Este projeto implementa um nó ROS2 que controla uma tartaruga em uma simulação do `turtlesim`. O nó pode comandar a tartaruga para se mover em um círculo e permite ao usuário interagir com a simulação por meio de comandos do teclado.

## Descrição

O nó `DriverNode` controla uma tartaruga no ambiente `turtlesim` da seguinte maneira:

- **Publicação de Velocidades**: Publica comandos de velocidade para fazer a tartaruga se mover em um círculo.
- **Serviços de Spawn e Kill**: Permite criar uma nova tartaruga e removê-la da simulação respectivamente.
- **Configuração do Lápis**: Define as propriedades do traço desenhado pela tartaruga.

Os usuários podem interromper a simulação pressionando 'Q'. Isso acionará a funcionalidade que mata a tartaruga antes de encerrar o programa após um atraso de 15 segundos.

## Pré-requisitos

Para usar este projeto, você precisará ter o seguinte instalado em seu sistema:

- ROS 2 (foi testado com a versão Humble Hawksbill)
- Python 3.8 ou superior
- turtlesim (que geralmente vem com instalações padrão do ROS 2)

## Instalação

Para executar este projeto, clone este repositório em seu ambiente ROS 2 workspace:

```bash
cd ~/dev_ws/src  # Ou qualquer que seja o seu workspace ROS 2
git clone https://github.com/AntonioArtimonte/Ponderada_ROS
```

Após clonar o repositório, você precisará compilar o código fonte usando:


```bash
cd ~/dev_ws
colcon build
```

Não se esqueça de fontar o ambiente ROS 2 após a construção:

```bash
. ~/dev_ws/install/setup.bash
```


