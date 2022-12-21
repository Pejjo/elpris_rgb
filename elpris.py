import hashlib
import requests
import time
import datetime
import serial

tsLastPriceData = 0
tsLastAverageData = 0
priceData=None
averageData=None

dim=10

def getPrice(action):
  #action can be now, today, plus8h, plus12h, date (day in timestamp), weekavg, 8dayavg (week+tomorrow), monthavg

  h = hashlib.new('ripemd160')
  ts = int(time.time())

  payload = {
    'seed': '',
    'timestamp': str(ts),
    'action': action,
    'area': 'SE3', 
  }

  for val in payload.values():
    h.update(val.encode())

  payload['key']=h.hexdigest()

  url = 'http://extra.hoj.nu/timpris/query.php'

  response = requests.get(url, params=payload)

  if (response.status_code==200):
    resp=response.json()
    data=resp['data']
    return data
  else:
    return None

# Return a RGB colour value given a scalar v in the range [vmin,vmax]
# In this case each colour component ranges from 0 (no contribution) to
# 1 (fully saturated), modifications for other ranges is trivial.
# The colour is clipped at the end of the scales if v is outside
# the range [vmin,vmax]
#

def GetColour(v, vmin, vmax):
   c = [1.0, 1.0, 1.0]

   if (v < vmin):
      v = vmin
   if (v > vmax):
      v = vmax
   dv = vmax - vmin

   if (v < (vmin + 0.25 * dv)):
      c[0] = 0
      c[1] = 4 * (v - vmin) / dv
   elif (v < (vmin + 0.5 * dv)):
      c[0] = 0;
      c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv
   elif (v < (vmin + 0.75 * dv)):
      c[0] = 4 * (v - vmin - 0.5 * dv) / dv
      c[2] = 0
   else:
      c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv
      c[2] = 0

   return c


def sendPriceArray(priceRatio, pwm1, pwm2, do1, do2):

  ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # open serial port
  print(ser.name)         # check which port was really used

  arr=[]

  for y in priceRatio:
    c=GetColour(y,0,2)
    c[0]=int(c[0]*dim)
    c[1]=int(c[1]*dim) 
    c[2]=int(c[2]*dim)
    print(c)
    arr.append(c)

  print(arr)

  data = [item for sublist in arr for item in sublist]

  data.insert(0, 0x02)
  data.append(pwm1)
  data.append(pwm2)
  data.append(do1)
  data.append(do2)
  data.append(0x03)

  print(data)
  ser.write(data)     # write a string
  resp=s = ser.read(80)
  print(resp)
  ser.close()             # close port
pwm=38
while(1):
  now=time.time()

  if priceData:
    for idx, itm in enumerate(priceData):
      if (itm['expTs'] < now):
        print("Popping index ", idx)
        priceData.pop(idx)
      print(itm, time.mktime(datetime.datetime.strptime(itm['time'], "%Y-%m-%d %H:%M:%S").timetuple()))


  if (priceData==None) or (tsLastPriceData < now) or len(priceData) < 8: 

    newData=getPrice('plus12h')
    print("Requesting new price", now)

    for itm in newData:
      itm['expTs']=time.mktime(datetime.datetime.strptime(itm['time'], "%Y-%m-%d %H:%M:%S").timetuple())+59*60+59
      print(itm, time.mktime(datetime.datetime.strptime(itm['time'], "%Y-%m-%d %H:%M:%S").timetuple()))

    if newData != None:
      print("Adding valid data")
      tsLastPriceData=time.mktime(datetime.datetime.strptime(newData[-8]['time'], "%Y-%m-%d %H:%M:%S").timetuple())
      priceData=newData
  else:
    print(tsLastPriceData, '>', now, "is valid another ",(tsLastPriceData-now))

  if (averageData==None) or (tsLastAverageData < now): 

    newData=getPrice('monthavg')
    print("Requesting new average", now, '>', tsLastAverageData)

    for itm in newData:
      print(itm, time.mktime(datetime.datetime.strptime(itm['start'], "%Y-%m-%d %H:%M:%S").timetuple()))

    if newData != None:
      print("Adding valid data")
      tsLastAverageData=now + 24*60*60
      averageData=newData
  else:
    print(tsLastAverageData, '>', now, " is valid another ", (tsLastAverageData-now))

  
  if (priceData != None) and (averageData != None):
    avg=float(averageData[0]['average'])
    priceRatio=[]
    for itm in priceData[:8]:
      priceRatio.append(float(itm['price'])/avg)

    print(priceRatio)
    sendPriceArray(priceRatio,pwm,255-pwm,0,0)
  pwm+=1
  if pwm>174:
    pwm=31
  time.sleep(2*5)
