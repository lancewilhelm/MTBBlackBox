#!/usr/bin/env python
# coding: utf-8

# In[1]:


import plotly.plotly as py
import plotly.graph_objs as go
import plotly.figure_factory as FF

import numpy as np
import pandas as pd


# In[2]:


df = pd.read_csv('data/Mon May  6 21_33_42 2019.csv')

sample_data_table = FF.create_table(df.head())
py.iplot(sample_data_table, filename='sample-data-table')

