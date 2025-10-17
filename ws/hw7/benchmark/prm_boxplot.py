#prm_boxplot.py
#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load benchmark results
df = pd.read_csv('prm_benchmark.csv')

# Filter only valid solutions for path length and time plots
valid_df = df[df['valid'] == 1]

# Boxplot: Path Length
plt.figure(figsize=(10,6))
sns.boxplot(x='n', y='path_length', hue='r', data=valid_df)
plt.title('PRM Path Length Boxplot (Valid Solutions Only)')
plt.ylabel('Path Length')
plt.xlabel('n')
plt.legend(title='r')
plt.tight_layout()
plt.savefig('prm_path_length_boxplot.png')
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
