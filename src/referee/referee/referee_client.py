import socket
import struct
import signal
import time

class TimeoutException(Exception):
    pass

def timeout(seconds=1.0, error_message="Function call timed out"):
    def decorator(func):
        def _handle_timeout(sigum, frame):
            raise TimeoutException(error_message)
        
        def wrapper(*args, **kwargs):
            # Separate the integer and fractional parts of seconds
            int_seconds = int(seconds)
            frac_seconds = seconds - int_seconds
            
            # Set the signal alarm for the integer part of seconds
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(int_seconds)
            
            try:
                if frac_seconds > 0:
                    # If there’s a fractional part, delay it using time.sleep in a separate thread
                    time.sleep(frac_seconds)
                
                result = func(*args, **kwargs)
            finally:
                # Disable the alarm after function execution
                signal.alarm(0)
                
            return result
        
        return wrapper
    
    return decorator

class Client:
    """Client that handles the UDP multicast communication for SSL referee messages."""

    def __init__(self, ip: str, port: int, buffer_size: int = 1024):
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self):
        """Sets up the multicast socket to receive data."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))

        mreq = struct.pack("4sl", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    @timeout(0.005)
    def receive(self):
        """Receive a message from the multicast group and return it as raw data."""
        try:
            data, _ = self.sock.recvfrom(self.buffer_size)
            return data
        except Exception as e:
            raise RuntimeError(f"Error receiving multicast message: {e}")