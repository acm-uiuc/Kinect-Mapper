import processing.serial.*;
Serial port;

String get_analog = "2/1/9/";  // 2 = read, 1 = analog, 9 = pin #, / = break
String data = "";

PFont font;

void setup()
{
  size(400,400);
  port = new Serial(this, Serial.list()[0], 9600);
  font = loadFont("LiberationSans-Bold-48");
  textFont(font, 200);
}

void draw()
{
  background(0,0,0);
  fill(46, 209, 2);
  text(data, 70, 175);
  fill(0, 102, 153);
  text("not in use", 70, 370);
}
  
void serialEvent (Serial port)
{
  port.write(get_analog);
  
  while (port.available() > 0) {
    data = port.readString();
  }
}
