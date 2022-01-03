import pickle
import sys
import pandas as pd

file_name = sys.argv[1]
file = open(str(file_name), 'rb')
dic = {}
with open(str(file_name), 'rb') as fr: 
	try: 
		while True:
			dic.update(pickle.load(fr))
	except EOFError: 
		pass

df = pd.DataFrame(dic).T
df.to_csv('unpickled.csv')
