'''
@File    :   try.py    

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/7/13 11:38   gw         1.0         None
'''
import pandas as pd
import numpy as np


s = "278 160 308 488 254 341 254 130 63 295 481 395 85 461 231 383 395 103 413 231 160 228 346 252 168 404 300 387 111 317"
s = s.split(" ")
r = []
for item in s:
    item = int(item)
    r.append(item)
print(r)

r = pd.date_range('6/14/2018','7/13/2018')
b = []
for i in range(30):
    a = str(r[i])
    t = a[6:10]
    b.append(t)
print(b)
