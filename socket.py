#!/usr/bin/env python3
"""
esp_client.py - Simple robust TCP client for talking to an ESP (ESP32/ESP8266) TCP server.

Features:
- Connects to host:port (TCP)
- Sends plain text or JSON (dict -> json)
- Receives response with timeout
- Auto-reconnect with backoff on failures
- Interactive mode to type messages live
- Small CLI for quick one-off sends
"""

import socket
import json
import time
import argparse
from typing import Optional, Union


class ESPClient:
    def __init__(
        self,
        host: str,
        port: int = 3333,
        timeout: float = 5.0,
        reconnect: bool = True,
        max_retries: int = 5,
        base_backoff: float = 1.0,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.reconnect = reconnect
        self.max_retries = max_retries
        self.base_backoff = base_backoff
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        """Establish a TCP connection (with retries if enabled)."""
        if self.sock:
            return
        attempt = 0
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self.timeout)
                s.connect((self.host, self.port))
                self.sock = s
                return
            except Exception as e:
                attempt += 1
                if not self.reconnect or attempt > self.max_retries:
                    raise ConnectionError(f"Failed to connect to {self.host}:{self.port}: {e}")
                backoff = self.base_backoff * (2 ** (attempt - 1))
                time.sleep(backoff)

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _ensure_connected(self) -> None:
        if not self.sock:
            self.connect()

    def send(self, data: Union[str, bytes, dict]) -> None:
        """Send data. If dict provided, send JSON string."""
        self._ensure_connected()
        if isinstance(data, dict):
            payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        elif isinstance(data, str):
            payload = data.encode("utf-8")
        else:
            payload = data
        try:
            # Send the raw bytes. Many ESP servers read until newline; add newline optionally in caller.
            self.sock.sendall(payload)
        except (BrokenPipeError, ConnectionResetError, socket.error):
            # Try reconnect once and resend
            self.close()
            if self.reconnect:
                self.connect()
                self.sock.sendall(payload)
            else:
                raise

    def recv(self, bufsize: int = 4096, timeout: Optional[float] = None) -> Optional[str]:
        """Receive data and return decoded string, or None on timeout/no-data."""
        if timeout is None:
            timeout = self.timeout
        self._ensure_connected()
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(bufsize)
            if not data:
                # Remote closed
                self.close()
                return None
            try:
                return data.decode("utf-8", errors="replace")
            except Exception:
                return data.hex()
        except socket.timeout:
            return None
        except (ConnectionResetError, OSError):
            self.close()
            return None

    def request(self, message: Union[str, dict], expect_response: bool = True, recv_timeout: Optional[float] = None) -> Optional[str]:
        """Send a message and optionally wait for a single response."""
        # Common pattern: many ESP code expects newline-terminated messages. Caller should add '\n' if needed.
        self.send(message)
        if expect_response:
            return self.recv(timeout=recv_timeout)
        return None

    def interactive(self, prompt: str = "> "):
        """Simple interactive loop: type a message, sends it, prints reply."""
        print(f"Interactive mode. Connected to {self.host}:{self.port}. Type 'quit' or Ctrl-C to exit.")
        try:
            while True:
                try:
                    line = input(prompt)
                except EOFError:
                    break
                if not line:
                    continue
                if line.lower() in ("quit", "exit"):
                    break
                # If user types valid JSON, send as JSON, else string (no extra newline added).
                out = None
                try:
                    parsed = json.loads(line)
                    out = parsed
                except Exception:
                    out = line
                try:
                    self.request(out)
                except Exception as e:
                    print(f"[send error] {e}")
                    # try to continue - try reconnect
                    try:
                        self.connect()
                    except Exception as e2:
                        print(f"[reconnect failed] {e2}")
                        time.sleep(1)
                        continue
                # Try to receive a response quickly (non-blocking-ish)
                resp = self.recv(timeout=1.0)
                if resp is None:
                    print("<no response>")
                else:
                    print(f"< {resp!s}")
        except KeyboardInterrupt:
            print("\nbye")
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(description="ESP TCP client (send text or JSON).")
    parser.add_argument("host", help="ESP host IP or hostname")
    parser.add_argument("-p", "--port", type=int, default=3333, help="TCP port (default 3333)")
    parser.add_argument("-m", "--message", help="Message to send (plain text). If omitted and not interactive, prints help.")
    parser.add_argument("--json", action="store_true", help="Treat message as JSON literal (sends parsed JSON).")
    parser.add_argument("-i", "--interactive", action="store_true", help="Interactive mode")
    parser.add_argument("-t", "--timeout", type=float, default=5.0, help="Socket timeout seconds")
    parser.add_argument("--no-reconnect", dest="reconnect", action="store_false", help="Disable auto-reconnect")
    args = parser.parse_args()

    client = ESPClient(args.host, port=args.port, timeout=args.timeout, reconnect=args.reconnect)

    if args.interactive:
        try:
            client.connect()
            client.interactive()
        except Exception as e:
            print(f"Connection error: {e}")
        return

    if args.message:
        payload = None
        if args.json:
            try:
                payload = json.loads(args.message)
            except Exception as e:
                print(f"Invalid JSON: {e}")
                return
        else:
            payload = args.message
        try:
            client.connect()
            # Many ESP sketches expect newline-terminated commands; add newline if you want:
            # payload = payload + "\n"  (only for str)
            resp = client.request(payload, expect_response=True, recv_timeout=3.0)
            if resp is None:
                print("<no response>")
            else:
                print(resp)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            client.close()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()