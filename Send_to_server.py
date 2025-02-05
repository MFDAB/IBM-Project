
import requests
import threading

def IBM_server(pic_name_ibm , pic_name_local , abs_path ): 
  url = "https://ef15ef30-157d-4e19-bc6c-3719ff27eb32.mock.pstmn.io/api/visual-inspection"

  payload={}
  files=[
    (pic_name_ibm,(pic_name_local,open(abs_path,'rb'),'image/jpg')) # change to whatever ur file is 
  ]
  headers = {}

  response = requests.request("POST", url, headers=headers, data=payload, files=files)

  print(response.text)