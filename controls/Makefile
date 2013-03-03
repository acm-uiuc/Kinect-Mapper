CC = g++
LINKOPTS = -pthread

OBJS = aux_socket.cpp

server: $(OBJS) server.cpp
	$(CC) $(LINKOPTS) -o server.out $(OBJS) server.cpp

client: $(OBJS) client.cpp
	$(CC) -o client.out $(OBJS) client.cpp

clean:
	rm -f *.o client.out server.out

