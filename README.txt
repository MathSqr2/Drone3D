1º passo é configurar o SSH no Raspberry. Fica a vosso critério a maneira como o fazer, o que interessa é que o Raspberry tem que ter um IP estático associado a si, porque vai ser necessário para mais tarde.
Usámos um computador para criar um Access Point à qual o Raspberry se liga automaticamente no boot.

A partir daqui toda a instalação de software e execução dos comandos que aparece aqui deve ser feita através do SSH, porque diretamente no Raspberry trouxe alguns problemas.

Fazer o download do Firmware do PX4 (poderá ser necessário fazer fork usando uma conta do github antes de fazer download, já não tenho a certeza)

git clone https://github.com/PX4/Firmware.git

Isto vai criar uma pasta chamada Firmware, que pode estar em qualquer diretório do Raspberry, não faz diferença.

A seguir vamos instalar o ROS. É necessário escolher uma distribuição de ROS. Atenção que após escolher uma distribuição, todas as instalações tem que estar associadas a essa distribuição. Nós escolhemos o ROS Melodic, por ser o mais recente.

Link para a instalação do ROS Melodic:
http://wiki.ros.org/melodic/Installation/Ubuntu

Instalar o MAVROS associado ao Melodic (caso use outra distribuição de ROS, é mudar o nome de melodic para o nome respetivo)

sudo apt-get install ros-melodic-mavros-msgs
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

Instalar dataset de coordenadas geográficas:

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

Agora vem a parte que pode ser mais chata entender. Para correr o código é necessário usar o catkin para a criação de um workspace no ROS, de um package que vai corresponder ao código a correr. A melhor maneira de perceber isto é ver os tutoriais no site do ROS, uma vez que não dá para simplificar mais o que está lá escrito.

Instalação do catkin (poderá dizer que já foi instalado, caso isso aconteça, é só ignorar este passo):
sudo apt-get install ros-melodic-catkin

Como criar e gerir um workspace no ROS:
http://wiki.ros.org/catkin/Tutorials

Após entenderem como criar um workspace, podem criar um package e corrê-lo ou usar o nosso (https://drive.google.com/drive/folders/18cD5W4r_kSBGtEZUFYxzoKlLx7pDW54y?usp=sharing). O código das missões está na pasta src dentro da pasta "Código Missão". Cada uma das pastas dentro do src corresponde a um package.
O package que foi usado na missão final do projeto de PEI foi o do package 'waypointpush2'

Antes de continuar, verifiquem o seguinte:
No PC que está a fazer de AP e aonde estão a usar o SSH, adicionem a seguinte linha ao fim do ficheiro do ~/.bashrc:

export ROS_MASTER_URI=http://192.168.12.168:11311

Onde 192.168.12.168 tem que ser substituído pelo IP estático no Raspberry. A porta deixem estar 11311

No Raspberry vão na mesma ao ficheiro ~/.bashrc e confirmem se no fim do ficheiro existe o seguinte, caso contrário adicionem o que está em falta:

source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.12.168:11311
export ROS_HOSTNAME=192.168.12.168
source /home/ubuntu/catkin_ws/devel/setup.bash

A primeira linha em vez de melodic poderá ser outro nome (correspondente ao nome da distribuição utilizada)
Na segunda e terceira linha, substituam na mesma o IP pelo do Raspberry (novamente deixem na segunda linha a porta como 11311)

A quarta linha tem que ser o diretório correspondente ao workspace criado por vocês (dentro do workspace existirá o diretório devel/setup.bash)

Se tudo estiver bem instalado e se o Raspberry estiver ligado diretamente ao Pixhawk4 já será possível correr o ficheiro de configuração do PX4 (pode ser corrido em qualquer diretório):
roslaunch mavros px4.launch

Se der erro, é porque faltou configurar/instalar algo.

Se estiver tudo já podem correr um package que queiram usando:
rosrun [nome do package] [nome do node (ficheiro) principal com o código]
e.g.
rosrun waypointpush2 waypointpush2_node

Agora vamos explicar resumidamente o código da missão que está no link disponibilizado acima no diretório Código Missão/src/waypointpush2/src/waypointpush2_node.cpp

A primeira parte código são as configurações dos tópicos a qual o MAVROS subscreve para mandar mensagens para o FCU e outras configurações, e por isso não interessa entrar em pormenores por aí.
O que interessa é a parte que tem o comentário // LOAD WAYPOINT'S LIST, porque é a parte que poderá ser editada.
Usámos uma função (não interessa como está feita), interessa é que ao chamá-la, ela carrega um conjunto de coordenadas para o sistema, aonde o drone irá seguir essa trajetória.

Basta então chamar a função, tantas vezes quanto o número de coordenadas que queremos.
A nível da maneira como se chama as funções, alterem apenas os seguintes argumentos (os restantes deixem como estão no exemplo do waypointpush2):
O 5º argumento pode ser mavros_msgs::CommandCode::NAV_TAKEOFF, mavros_msgs::CommandCode::NAV_WAYPOINT ou mavros_msgs::CommandCode::NAV_LAND conforme a necessidade.
Os últimos 7 argumentos variam conforme o comando que quer utilizar (takeoff, waypoint ou land) e a informação sobre estes pode ser encontrada nos seguintes links:
Para qualquer um destes seguintes comandos, destes 7 argumentos, os últimos 3 são sempre altitude, longitude e altitude, respetivamente.

Para o takeoff:
https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF

Para o waypoint:
https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT

Para o land:
https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND

Alterem conforme preferirem para a missão que querem realizar.
Depois de qualquer alteração ao ficheiro, não se esqueçam (tal como está escrito nos tutoriais do catkin) de ir ao diretório pai do workspace e fazer catkin_make para compilar tudo.
Depois basta fazer, tal como foi mencionado antes:

rosrun [nome do package] [nome do node/ficheiro]

Apenas devem executar este comando, depois de já terem feito roslaunch mavros px4.launch (porque este package é que carrega as configurações do PX4 e por isso tem que ser executado antes).
Para executar ambos, vão precisar de 2 terminais diferentes cada terminal executa o seu comando.

Se tudo estiver bem, após executar o rosrun e, supondo que o drone já tem um fix no GPS, este demora uns segundos até começar a missão.

Para a execução dos sensores, que irão ser lidos através por I2C (no nosso caso porta x77), foi criado um package que contém um talker e um listener:
- o talker usa o módulo demo.py para ler os valores do sensor e publicar num tópico
- o listener escuta esse tópico mais os tópicos publicados pelo mavros que publicitam as posições GPS, organiza a informação no formato "x;y;z;temperatura;pressão;humidade"
que será depois guardado para pós processamento em unity.
Para execução destes ficheiros fazemos: rosrun sensor_test talker.py    (para o talker)
                                        rosrun sensor_test listener.py  (para o listener)
