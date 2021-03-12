/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 * Curso IntRob 2019                                                   *
 *                                                                     *
 * Ejemplo de Interfaz de control manual                               *
 *                                                                     *
 * MLG:
 * 20200203
 * Se agregan cambios para controlar Bebop2. Se usan como
 * referencias a teleop_twist_keyboard.py y al codigo de Quetzal_doc
 * de un curso con la UPAEP.
 * OJO teleop_twist_keyboard.py tiene mal documentadas las teclas;
 * algunas no se usan y otras estan intercambiadas.
 * OJO Quetzal_doc se usa solo una variable Twist tanto para el control
 * del Bebop 2 como para su camara. Ademas, estan separados los eventos
 * del teclado en un nodo ROS independiente de los comandos del Bebop.
 * Al parecer hay confucion en los nombres de 'pitch' y 'roll', ya que esos
 * son para angulos, pero en el twist se usan para la velocidad lineal 'x' y
 * 'y' respectivamente.
 ***********************************************************************/
 
#include <QKeyEvent>
#include "teclado.h"

#define BEBOP2

KeyPress::KeyPress(QWidget *parent): 
		nh_("~"), QWidget(parent)
{
	ROS_INFO("Init Keyboard Controller");
	
#ifdef BEBOP2
    pubTakeoff1_ = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 3); // Bebop 2
    pubLand1_ = nh_.advertise<std_msgs::Empty>("/bebop/land", 3); // Bebop 2
    pubCommandPilot1_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 3); // Bebop 2
    CameraPub = nh_.advertise<geometry_msgs::Twist>("/bebop/camera_control", 3); // Bebop 2
#else
    pubTakeoff1_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 3); // tum_simulator
    pubLand1_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 3); // tum_simulator
    pubCommandPilot1_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3); // tum_simulator
#endif

    OverridePub = nh_.advertise<std_msgs::Int8>("/keyboard/override", 3);
	
    // MAV's control
	speed = 0.3;
	altitude_speed = 1.0;
	pitch = 0;
	roll = 0;
	yaw = 0;
	altitude = 0;

    // MAV's camera control
    CameraAngY = 0;
	
    // Auto-mode
	override.data = 0;


    // User Interface
	Speed = new QLabel("Pitch", this);
	Alt_speed = new QLabel("Roll", this);
	Fovea_y = new QLabel("Fovea", this);
	
	Speed_value = new QLabel("0", this);
	Alt_speed_value = new QLabel("0", this);
	Fovea_value_y = new QLabel("  0", this);
	Command = new QLabel(" ", this);

    font = Command->font();
	font.setPointSize(20);
	font.setBold(true);
	Command->setFont(font);
	
    Speed->setText("Speed:");
    Alt_speed->setText("Altitude Speed:");
	Fovea_y->setText("Fovea Y:");
	
	grid = new QGridLayout(this);
	
	grid->addWidget(Speed, 1, 0);
	grid->addWidget(Alt_speed, 2, 0);
	grid->addWidget(Fovea_y, 4, 0);
		
	grid->addWidget(Speed_value, 1 ,1);
	grid->addWidget(Alt_speed_value, 2, 1);
	grid->addWidget(Fovea_value_y, 4, 1);
	grid->addWidget(Command, 5, 0);

    Speed_value->setText(QString::number(speed));
    Alt_speed_value->setText(QString::number(altitude_speed));

    setLayout(grid);
}

KeyPress::~KeyPress()
{

}

void KeyPress::keyPressEvent(QKeyEvent *event) 
{
	bool send_command = true;

	commandPilot.linear.x = 0.0;
	commandPilot.linear.y =  0.0;
	commandPilot.linear.z = 0.0;
	commandPilot.angular.z = 0.0;
    commandPilot.angular.x = 0.0;
    commandPilot.angular.y = 0.0;
	
	pitch = 0;
	roll = 0;
	altitude = 0;
	yaw = 0;

	if(event->key() == Qt::Key_H)
	{	
		pitch = 0.0;
		roll = 0.0;
		altitude = 0.0;
		yaw = 0.0;
		Command->setText("HOVERING");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_T)
	{	
	
		pubTakeoff1_.publish(msgTakeoff);
		Command->setText("TAKE OFF");
		setLayout(grid);
	
	}
	else if(event->key() == Qt::Key_Space)
	{	

		pubLand1_.publish(msgLand);
		Command->setText("LANDING");
		setLayout(grid);

	}
	else if(event->key() == Qt::Key_X)
	{	
		override.data = 6;
        OverridePub.publish(override);
		send_command = false;
		Command->setText("FORMATION");
		setLayout(grid);
		
	}
	else if(event->key() == Qt::Key_C)
	{	
		override.data = 10;
        OverridePub.publish(override);
		send_command = false;
		Command->setText("<font color=red>CANCEL CONTROLLER</font></h2>");
		setLayout(grid);

	}	
	else if(event->key() == Qt::Key_Q)
	{	
		yaw += speed;
		Command->setText("YAW LEFT");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_E)
	{	
		yaw -= speed;
		Command->setText("YAW RIGHT");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_W)
	{	
		pitch += speed;
		Command->setText("FORWARD");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_S)
	{	
		pitch -= speed;
		Command->setText("BACKWARD");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_A)
	{	
		roll += speed;
		Command->setText("LEFT");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_D)
	{	
		roll -= speed;
		Command->setText("RIGHT");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_Up)
	{	
		altitude += altitude_speed;
		Command->setText("UP");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_Down)
	{	
		altitude -= altitude_speed;
		Command->setText("DOWN");
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_Left)
	{	
		speed -= .1;
        if(speed < .1)
            speed = 0.1;
		Speed_value->setText(QString::number(speed));		
		setLayout(grid);
	}
	else if(event->key() == Qt::Key_Right)
	{	
		speed += .1;
        if(speed > 1)
            speed = 1;
		Speed_value->setText(QString::number(speed));
		setLayout(grid);
	}
    else if(event->key() == Qt::Key_I) {
        CameraAngY += 5;
        CameraTwist.angular.y = CameraAngY;
        CameraPub.publish(CameraTwist);

        Command->setText("CAMERA UP");
        setLayout(grid);
    }
    else if(event->key() == Qt::Key_K) {
        CameraAngY -= 5;
        CameraTwist.angular.y = CameraAngY;
        CameraPub.publish(CameraTwist);

        Command->setText("CAMERA DOWN");
        setLayout(grid);
    }
	
	if(event->key() == Qt::Key_Escape) 
	{
		qApp->quit();
	}
	
    commandPilot.linear.x = pitch; // x?
    commandPilot.linear.y =  roll; // y?
    commandPilot.linear.z = altitude; // z?
	commandPilot.angular.z = yaw;
	
	pubCommandPilot1_.publish(commandPilot);
}



