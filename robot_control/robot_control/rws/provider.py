"""Thin HTTP layer over ABB Robot Web Services. Ported from the diploma
project's intranodes_pkg/robot_controller_provider.py."""

import requests
from requests.auth import HTTPBasicAuth
from robot_control.exceptions import RWSException
requests.packages.urllib3.disable_warnings(
    requests.packages.urllib3.exceptions.InsecureRequestWarning
)

class RWSClient:

    def __init__(self, host: str, username: str, password: str, port=80, logger=None):
        proto = "https"
        self.base_url = f"{proto}://{host}:{port}"
        self.session = requests.Session()
        self._logged_in = False
        self.timeout_sec = 2  # seconds
        self.logger = logger

        self.session.verify = False
        self.auth_method = HTTPBasicAuth(username, password)
        self.header_typ = {'Accept': 'application/hal+json;v=2.0', 
                       'Content-Type': 'application/x-www-form-urlencoded;v=2.0'}
        self.header_opt = {'Accept': 'application/xhtml+xml;v=2.0'}


    def login(self):
        """Log in to the ABB RWS server. Returns True on success."""

        self._logged_in = False
        url = f"{self.base_url}"
        try:
            resp = self.session.get(url, headers=self.header_typ, auth=self.auth_method, timeout=self.timeout_sec)

            if 'ABBCX' not in self.session.cookies.get_dict():
                self.logger.error("Login failed: missing ABBCX cookie")
                return False

            if resp.status_code == 200:
                self._logged_in = True
                self.logger.info(f"Login successful, status code: {resp.status_code}")
                return True
            else:
                self.logger.info(f"Login failed, status code: {resp.status_code}")
                return False
            
        except Exception as e:
            self.logger.error(f"Login request failed, check connection: {e}")
            raise RWSException(f"Login request failed: {e}") from e

    
    def logout(self):
        """Log out and close the session. Returns True on success."""
        url = f"{self.base_url}/logout"
        self._logged_in = False
        try:
            # Check if session is already closed
            if not self.session.adapters:
                self.logger.info("Session already closed.")
                return False

            resp = self.session.get(url, headers=self.header_typ, timeout=self.timeout_sec)
            self.session.close()

            if resp.status_code == 204:
                self.logger.info(f"Logout successful, status code: {resp.status_code}")
                return True
            else:
                self.logger.info(f"Logout failed (probably already logged out), status code: {resp.status_code}")
                return False
            
        except Exception as e:
            self.logger.error(f"Logout request failed, message: {e}")
            raise RWSException(f"Logout request failed: {e}") from e
        
    def get_login_state(self):
        """Check if the session is still logged in."""
        url = f"{self.base_url}"
        try:
            resp = self.session.get(url, headers=self.header_typ, timeout=self.timeout_sec)
            if resp.status_code == 200:
                return True
            else:
                return False
            
        except Exception as e:
            self.logger.error(f"Login state request failed, message: {e}")
            raise RWSException(f"Login state request failed: {e}") from e

        
    def send_keepalive(self):
        """Send a lightweight GET to keep the connection alive."""

        # Lightweight GET request - just check controller state
        url = f"{self.base_url}/rw/system"
        try:
            resp = self.session.get(url, headers=self.header_typ, timeout=self.timeout_sec)
            if resp.status_code == 200:
                self.logger.info("Keepalive successful")
                self._logged_in = True
                return True
            else:
                self.logger.error(f"Keepalive failed, status code: {resp.status_code}")
                self._logged_in = False
                return False
            
        except Exception as e:
            self.logger.error(f"Keepalive request failed: {e}")
            self._logged_in = False
            raise RWSException(f"Keepalive request failed: {e}") from e

    
    def get_request(self, path):
        """Send a GET request. Returns (json_data, status_code)."""
        
        url = f"{self.base_url}{path}"
        try:
            resp = self.session.get(url, headers=self.header_typ, timeout=self.timeout_sec)
            if resp.status_code != 200:
                self.logger.error(f"GET {path} failed: {resp.status_code}")
            
            return (resp.json() if resp.content else None, resp.status_code)
        except Exception as e:
            self.logger.error(f"GET request {path} failed: {e}")
            return (None, int(-1))

    def post_request(self, path, dataIn=None):
        """Send a POST request. Returns the HTTP status code."""

        url = f"{self.base_url}{path}"
        try:
            resp = self.session.post(url, headers=self.header_typ, data=dataIn, timeout=self.timeout_sec)
            if resp.status_code not in (200, 201, 204, 500): # 500 is returned by some DIPC calls
                self.logger.error(f"POST {path} failed: {resp.status_code}")
            return resp.status_code
        
        except Exception as e:
            self.logger.error(f"POST request {path} failed: {e}")
            return int(-1)
    



    def options_request(self, path):
        """Send an OPTIONS request. Returns (json_data, status_code)."""

        url = f"{self.base_url}{path}"

        try:
            resp = self.session.options(url, headers=self.header_opt)
            if resp.status_code not in (200, 201, 204):
                self.logger.error(f"OPTIONS {path} failed: {resp.status_code}")
            return (resp.json() if resp.content else None, resp.status_code)
    

        except Exception as e:
            self.logger.error(f"OPTIONS request {path} failed: {e}")
            return (None, int(-1))

    
    