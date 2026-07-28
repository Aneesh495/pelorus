package pelorus_server

import "core:fmt"
import "core:net"
import "core:os"
import "core:strings"
import "core:sync"

//
// Telemetry Server Configuration.
//
ServerConfig :: struct {
	port:        int,
	static_dir:  string,
	max_clients: int,
}

//
// Default server configuration.
//
server_config_default :: proc "contextless" () -> ServerConfig {
	return ServerConfig{
		port        = 8080,
		static_dir  = "web",
		max_clients = 32,
	}
}

//
// Active connected client connection.
//
ClientConnection :: struct {
	sock:       net.TCP_Socket,
	is_sse:     bool,
	created_at: f64,
}

//
// Embedded HTTP and Real-Time Telemetry Broadcaster.
//
TelemetryServer :: struct {
	config:         ServerConfig,
	listener:       net.TCP_Socket,
	clients:        [dynamic]ClientConnection,
	latest_json:    string,
	is_listening:   bool,
	mutex:          sync.Mutex,
}

//
// Initialize and bind telemetry server socket.
//
telemetry_server_init :: proc(srv: ^TelemetryServer, cfg: ServerConfig) -> bool {
	srv.config = cfg
	srv.clients = make([dynamic]ClientConnection, 0, cfg.max_clients)
	srv.latest_json = "{\"status\": \"initializing\"}"
	srv.is_listening = false

	endpoint := net.Endpoint{
		address = net.IP4_Any,
		port    = cfg.port,
	}

	listener, err := net.listen_tcp(endpoint, 32)
	if err != nil {
		fmt.eprintfln("[TelemetryServer] Failed to listen on port %d: %v", cfg.port, err)
		return false
	}

	// Set non-blocking mode so perception loop is never stalled
	net.set_blocking(listener, false)
	srv.listener = listener
	srv.is_listening = true
	fmt.printfln("[TelemetryServer] Online and listening at http://127.0.0.1:%d", cfg.port)
	return true
}

//
// Shut down and clean up server resources.
//
telemetry_server_destroy :: proc(srv: ^TelemetryServer) {
	sync.lock(&srv.mutex)
	defer sync.unlock(&srv.mutex)

	for c in srv.clients {
		net.close(c.sock)
	}
	delete(srv.clients)

	if srv.is_listening {
		net.close(srv.listener)
		srv.is_listening = false
	}
}

//
// Update cached perception state JSON string for HTTP queries.
//
telemetry_server_update_state :: proc(srv: ^TelemetryServer, json_state: string) {
	sync.lock(&srv.mutex)
	defer sync.unlock(&srv.mutex)
	srv.latest_json = json_state
}

//
// Broadcast latest perception state to all connected SSE clients.
//
telemetry_server_broadcast :: proc(srv: ^TelemetryServer, json_payload: string) {
	sync.lock(&srv.mutex)
	defer sync.unlock(&srv.mutex)

	sse_msg := fmt.tprintf("data: %s\n\n", json_payload)
	msg_bytes := transmute([]u8)sse_msg

	active_count := 0
	for i in 0 ..< len(srv.clients) {
		client := srv.clients[i]
		if !client.is_sse {
			srv.clients[active_count] = client
			active_count += 1
			continue
		}

		_, send_err := net.send_tcp(client.sock, msg_bytes)
		if send_err == nil {
			srv.clients[active_count] = client
			active_count += 1
		} else {
			net.close(client.sock)
		}
	}

	resize(&srv.clients, active_count)
}

//
// Non-blocking poll for incoming connections and HTTP requests.
//
telemetry_server_poll :: proc(srv: ^TelemetryServer) {
	if !srv.is_listening do return

	// Accept pending connections
	for {
		client_sock, _, err := net.accept_tcp(srv.listener)
		if err != nil do break

		net.set_blocking(client_sock, false)
		handle_incoming_connection(srv, client_sock)
	}
}

//
// Handle individual incoming HTTP request.
//
handle_incoming_connection :: proc(srv: ^TelemetryServer, client_sock: net.TCP_Socket) {
	req_buf: [2048]u8
	bytes_read, recv_err := net.recv_tcp(client_sock, req_buf[:])
	if recv_err != nil || bytes_read <= 0 {
		net.close(client_sock)
		return
	}

	req_text := string(req_buf[:bytes_read])
	lines := strings.split_lines(req_text)
	defer delete(lines)

	if len(lines) == 0 {
		net.close(client_sock)
		return
	}

	request_line := strings.trim_space(lines[0])
	tokens := strings.fields(request_line)
	defer delete(tokens)

	if len(tokens) < 2 || tokens[0] != "GET" {
		net.close(client_sock)
		return
	}

	path := tokens[1]

	// Route 1: SSE real-time stream (/stream)
	if path == "/stream" {
		header := "HTTP/1.1 200 OK\r\n" +
			"Content-Type: text/event-stream\r\n" +
			"Cache-Control: no-cache\r\n" +
			"Connection: keep-alive\r\n" +
			"Access-Control-Allow-Origin: *\r\n\r\n"
		net.send_tcp(client_sock, transmute([]u8)header)

		sync.lock(&srv.mutex)
		append(&srv.clients, ClientConnection{
			sock       = client_sock,
			is_sse     = true,
			created_at = 0.0,
		})
		sync.unlock(&srv.mutex)
		return
	}

	// Route 2: Snapshot JSON (/snapshot or /state.json)
	if path == "/snapshot" || path == "/state.json" || path == "/api/state" {
		sync.lock(&srv.mutex)
		json_payload := srv.latest_json
		sync.unlock(&srv.mutex)

		resp := fmt.tprintf(
			"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: %d\r\n\r\n%s",
			len(json_payload),
			json_payload,
		)
		net.send_tcp(client_sock, transmute([]u8)resp)
		net.close(client_sock)
		return
	}

	// Route 3: Static files (index.html, styles, scripts)
	file_to_serve := "web/index.html"
	content_type := "text/html; charset=utf-8"

	if path == "/" || path == "/index.html" {
		file_to_serve = "web/index.html"
		content_type = "text/html; charset=utf-8"
	} else if strings.has_suffix(path, ".js") {
		file_to_serve = fmt.tprintf("web%s", path)
		content_type = "application/javascript"
	} else if strings.has_suffix(path, ".css") {
		file_to_serve = fmt.tprintf("web%s", path)
		content_type = "text/css"
	}

	data, read_err := os.read_entire_file_from_path(file_to_serve, context.allocator)
	if read_err == nil {
		defer delete(data)
		resp_hdr := fmt.tprintf(
			"HTTP/1.1 200 OK\r\nContent-Type: %s\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: %d\r\n\r\n",
			content_type,
			len(data),
		)
		net.send_tcp(client_sock, transmute([]u8)resp_hdr)
		net.send_tcp(client_sock, data)
	} else {
		not_found := "HTTP/1.1 404 Not Found\r\nContent-Length: 13\r\n\r\n404 Not Found"
		net.send_tcp(client_sock, transmute([]u8)not_found)
	}

	net.close(client_sock)
}
