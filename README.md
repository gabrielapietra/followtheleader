<h2 align="center"> Follow the leader </h2>

O projeto consiste no desafio de "Seguir o mestre", onde um dos robô é considerado o mestre, sendo controlado pelo teleop, e o outro robô deverá seguí-lo no mapa.

<h3>Configurações iniciais </h3>

Foi utilizado o turtlebot3, de modo que foi usado os seguintes recursos dele, inseridos na pasta src:
```
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

<h3>Compilando</h3>

Esse comando foi feito na pasta principal, anterior à src.
```
catkin build
```


<h3>Inicializando o master</h3>

```
roscore
```

<h3>Definindo o robô que será utilizado</h3>

```
export TURTLEBOT3_MODEL=burger
```

<h3>Buildando e setando variáveis</h3>

```
source devel/setup.bash
catkin build
```

<h3>Rodando o mapa com dois robôs</h3>

```
roslaunch turtlebot3_gazebo two_tb3.launch
```

Nesse arquivo é possível ajustar a posição de onde os robôs iniciarão.

<h3>Seguindo o mestre</h3>

```
rosrun follow_me tb3_follow_node
```

<h3>Controlando o mestre</h3>
Pelo <i>teleop</i> é possível controlar o mestre pelo teclado.

```
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

