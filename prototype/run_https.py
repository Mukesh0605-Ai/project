import http.server
import ssl
import socketserver
import os

PORT = 3000
DIRECTORY = "."

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

# Generate a temporary self-signed cert if it doesn't exist (using OpenSSL)
if not os.path.exists("key.pem") or not os.path.exists("cert.pem"):
    print("Generating temporary self-signed certificate...")
    os.system('openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj "/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost"')

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    try:
        context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")
        httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
        print(f"Serving HTTPS on port {PORT} (https://localhost:{PORT})")
        print("Note: Your browser will show a security warning because this is a self-signed certificate. You can safely proceed for local testing.")
        httpd.serve_forever()
    except Exception as e:
        print(f"Failed to start HTTPS server: {e}")
        print("Please ensure openssl is installed and in your PATH.")
