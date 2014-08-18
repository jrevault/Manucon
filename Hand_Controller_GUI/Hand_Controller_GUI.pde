import processing.serial.*;

Serial myPort;  // Create object from Serial class

int min = 0;
int max = 1024;
float roll = (max-min)/2;
float pitch = (max-min)/2;
float throttle = 0;
String portName;
String raw = "";
String[] rawComponents = new String[3];


void setup() {
  size(1000,700);
  portName = Serial.list()[Serial.list().length-1];
  println("Opening serial port:  " + portName);
  myPort = new Serial(this, portName, 115200);
  
}

void draw() {
  background(50);
  while ( myPort.available() > 0) {  // If data is available,
    raw = myPort.readString().trim();         // read it and store it in val
    //println(raw);
  }
  if (!raw.equals("") && !raw.equals("\n")) {
      rawComponents = raw.split("x");
      //print(rawComponents);
      //println("  -   " + rawComponents.length);
      try {
        print("Parsing...");
        if (!rawComponents[0].equals(null)) { 
          roll = Float.parseFloat(rawComponents[0].trim());
          print(" ...roll... ");
        }
        else print( " fail ");
        if (!rawComponents[1].equals(null)) { 
          pitch = Float.parseFloat(rawComponents[1].trim());
          print(" ...pitch... ");
        }
        else print( " fail ");
        if (!rawComponents[2].equals(null)) { 
          throttle = Float.parseFloat(rawComponents[2].trim());
          println(" ...throttle... ");
        }
        else println( " fail ");
      }
      catch(NumberFormatException e) {
        println("CAUGHT");
      }
      
    
  }
    
  
  
  //draws the gimbal
  strokeWeight(3);
  stroke(150);
  line(800,50,800,350);
  line(650,200,950,200);
  noFill();
  strokeWeight(5);
  stroke(255);
  rect(650,50,300,300);
  ellipse(650+map(roll,1000,0,300,0),50+map(pitch,1000,0,0,300),30,30);
  
  
  //draws the two dials
  ellipse(715,450,130,130);
  ellipse(885,450,130,130);
  textAlign(CENTER,TOP);
  textSize(20);
  text("ROLL",715,520);
  text("PITCH",885,520);

  translate(715,450);
  rotate(radians(map(roll,0,1000,-45,45)));
  line(0,0,65,0);
  line(0,0,-65,0);
  line(0,0,0,-65);
  rotate(-radians(map(roll,0,1000,-45,45)));
  
  translate(170,0);
  rotate(radians(map(pitch,0,1000,-45,45)));
  line(0,0,65,0);
  line(0,0,-65,0);
  line(0,0,0,-65);
  rotate(-radians(map(pitch,0,1000,-45,45)));
  translate(-885,-450);
  
  rect(550,50,20,300);
  translate(540,50);
  rect(0,map(throttle,1000,0,0,300),40,20);
  translate(-540,-50);
  
  text(roll, 150, 200);
  text(pitch, 250,200);
  text(throttle, 350,200);
  
  textAlign(LEFT);
  text("Serial Port: " + portName, 50,50);
  
  
}
