import time
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

red = []
ir  = []
plt.style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

plt.tight_layout();

def main():

	def animate(i):
		plt.cla();
		if(red and ir):
			plt.plot(red[-500:-1] , label='red' , linewidth=0.5)
			plt.plot(ir[-500:-1] , label='ir' , linewidth=0.5)
			plt.legend(loc='upper left')
		plt.tight_layout();

	def on_message(client, userdata, message):
		for i in range(0, int(len(message.payload)/4)):
			red.append((message.payload[i*4]<<8) + message.payload[i*4 + 1])
			ir.append((message.payload[i*4 + 2]<<8) + message.payload[i*4 + 3])
			print(red[-1] , '\t'  , ir[-1])

	client = mqtt.Client();
	client.connect("broker.hivemq.com");
	client.on_message=on_message

	ani = animation.FuncAnimation(plt.gcf(), animate, interval=1000)

	client.subscribe("oximeter",0)
	while(1) :
		try :
			client.loop_start()
			plt.show();
			client.loop_stop()
		except KeyboardInterrupt:
			exit()

if __name__ == '__main__':
	main()