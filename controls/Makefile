CC = g++

OBJS = aux_socket.cpp

server: $(OBJS) server.cpp
	$(CC) -o server.out $(OBJS) server.cpp

client: $(OBJS) client.cpp
	$(CC) -o client.out $(OBJS) client.cpp

clean:
	rm -f *.o client.out server.out

