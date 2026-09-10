import json
import threading
import unittest
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
import server

class HostOwnershipTests(unittest.TestCase):
 def test_replaced_browser_cannot_overwrite_current_input(self):
  server.host=server.Host()
  http=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
  thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
  def post(path,body):
   request=urllib.request.Request(f'http://127.0.0.1:{http.server_port}'+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
   with urllib.request.urlopen(request) as response:return json.load(response)
  try:
   for client in ['first','current']:post('/configure',{'clientId':client,'track':'technical_test_circuit'})
   with self.assertRaises(urllib.error.HTTPError) as denied:post('/command',{'clientId':'first','running':True,'throttle':1})
   self.assertEqual(denied.exception.code,409);denied.exception.close();self.assertFalse(server.host.running)
   post('/command',{'clientId':'current','running':True,'throttle':.5})
   self.assertTrue(server.host.running)
   with self.assertRaises(urllib.error.HTTPError) as missing:post('/command',{'running':False,'throttle':0})
   missing.exception.close()
   self.assertTrue(server.host.running)
  finally:
   http.shutdown();http.server_close();thread.join();server.host.native.close()
if __name__=='__main__':unittest.main()
