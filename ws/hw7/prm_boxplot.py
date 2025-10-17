# Add local module directories to sys.path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../scripts'))

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Load benchmark results

csv_path = os.path.join(os.path.dirname(__file__), 'prm_benchmark.csv')# Load benchmark results

df = pd.read_csv(csv_path)
df = pd.read_csv('prm_benchmark.csv')



# Only plot valid solutions# Filter only valid solutions for path length and time plots

df_valid = df[df['valid'] == True]
valid_df = df[df['valid'] == 1]



# Create boxplot for path length by (n, r)# Boxplot: Path Length

df_valid.loc[:, 'n_r'] = df_valid['n'].astype(str) + ', r=' + df_valid['r'].astype(str)
plt.figure(figsize=(10,6))

plt.figure(figsize=(12, 6))
sns.boxplot(x='n', y='path_length', hue='r', data=valid_df)

sns.boxplot(x='n_r', y='path_length', data=df_valid)
plt.title('PRM Path Length Boxplot (Valid Solutions Only)')

plt.xticks(rotation=45)
plt.ylabel('Path Length')

plt.title('PRM Path Lengths by (n, r)')
plt.xlabel('n')

plt.xlabel('(n, r)')
plt.legend(title='r')

plt.ylabel('Path Length')
plt.tight_layout()

plt.tight_layout()
plt.savefig('prm_path_length_boxplot.png')

plt.savefig(os.path.join(os.path.dirname(__file__), 'prm_boxplot.png'))
plt.show()

plt.show()

# Boxplot: Computation Time
plt.figure(figsize=(10,6))
sns.boxplot(x='n', y='time_ms', hue='r', data=valid_df)
plt.title('PRM Computation Time Boxplot (Valid Solutions Only)')
plt.ylabel('Time (ms)')
plt.xlabel('n')
plt.legend(title='r')
plt.tight_layout()
plt.savefig('prm_time_boxplot.png')
plt.show()

# Barplot: Number of Valid Solutions
valid_counts = df.groupby(['n','r'])['valid'].sum().reset_index()
plt.figure(figsize=(10,6))
sns.barplot(x='n', y='valid', hue='r', data=valid_counts)
plt.title('Number of Valid PRM Solutions (out of 100)')
plt.ylabel('Valid Solutions')
plt.xlabel('n')
plt.legend(title='r')
plt.tight_layout()
plt.savefig('prm_valid_count_barplot.png')
plt.show()
