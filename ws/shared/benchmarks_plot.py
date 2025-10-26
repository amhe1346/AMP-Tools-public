# benchmarks_plot.py
import pandas as pd
import matplotlib.pyplot as plt

csv_files = [
    'build/bin/benchmark_results_m2.csv',
    'build/bin/benchmark_results_m3.csv',
    'build/bin/benchmark_results_m5.csv',
    'build/bin/benchmark_results_m6.csv'
]

dfs = []
for f in csv_files:
    try:
        df_temp = pd.read_csv(f)
        if not df_temp.empty:
            dfs.append(df_temp)
    except Exception as e:
        print(f"Skipping {f}: {e}")

if not dfs:
    print("No valid benchmark CSV files found.")
    exit(1)

df = pd.concat(dfs, ignore_index=True)
df['time'] = pd.to_numeric(df['time'], errors='coerce')
df['tree_size'] = pd.to_numeric(df['tree_size'], errors='coerce')

# Extract agent_count from filename and add as column
df_list = []
import re
for f in csv_files:
	match = re.search(r"benchmark_results_m(\d+)\.csv", f)
	if match:
		agent_count = int(match.group(1))
		df_temp = pd.read_csv(f)
		df_temp['agent_count'] = agent_count
		df_list.append(df_temp)
df = pd.concat(df_list, ignore_index=True)
df['time'] = pd.to_numeric(df['time'], errors='coerce')
df['tree_size'] = pd.to_numeric(df['tree_size'], errors='coerce')

# Filter for central planner only
central_df = df[df['planner'] == 'central']

# Group data by agent_count
agent_counts = sorted(central_df['agent_count'].dropna().unique())
time_data = [central_df[central_df['agent_count'] == m]['time'].dropna() for m in agent_counts]
tree_size_data = [central_df[central_df['agent_count'] == m]['tree_size'].dropna() for m in agent_counts]
labels = [f"m={int(m)}" for m in agent_counts]

# Calculate average computation time and tree size for each agent count
avg_times = [td.mean() for td in time_data]
avg_tree_sizes = [tsd.mean() for tsd in tree_size_data]

# Boxplot for computation time and tree size by agent count
plt.figure(figsize=(14, 8))
plt.subplot(1, 2, 1)
plt.boxplot(time_data, labels=labels)
plt.title('central planer box plot of computation time by number of agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Computation Time (seconds)')
plt.legend(["Computation Time"], loc="upper right")

plt.subplot(1, 2, 2)
plt.boxplot(tree_size_data, labels=labels)
plt.title('central planer box plot of tree size by number of agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Tree Size (total waypoints)')
plt.legend(["Tree Size"], loc="upper right")

plt.suptitle('central planer box plot of computation time and tree size by number of agents')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('central_planner_combined_boxplot.png')
plt.show()


print('Combined boxplot for central planner saved as central_planner_combined_boxplot.png.')
plt.show()

# Plot average computation time vs. number of agents (central)
plt.figure(figsize=(8,6))
plt.plot(agent_counts[:len(avg_times)], avg_times, marker='o')
plt.title('Central: Average Computation Time vs. Number of Agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Average Computation Time (seconds)')
plt.grid(True)
plt.savefig('central_avg_time_vs_agents.png')
plt.show()

# Plot average tree size vs. number of agents (central)
plt.figure(figsize=(8,6))
plt.plot(agent_counts[:len(avg_tree_sizes)], avg_tree_sizes, marker='o')
plt.title('Central: Average Tree Size vs. Number of Agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Average Tree Size (total waypoints)')
plt.grid(True)
plt.savefig('central_avg_tree_size_vs_agents.png')
plt.show()

# --- Decentral planner plotting ---
decentral_df = df[df['planner'] == 'decentral']
agent_counts_dec = sorted(decentral_df['agent_count'].dropna().unique())
time_data_dec = [decentral_df[decentral_df['agent_count'] == m]['time'].dropna() for m in agent_counts_dec]
tree_size_data_dec = [decentral_df[decentral_df['agent_count'] == m]['tree_size'].dropna() for m in agent_counts_dec]
labels_dec = [f"m={int(m)}" for m in agent_counts_dec]
avg_times_dec = [td.mean() for td in time_data_dec]
avg_tree_sizes_dec = [tsd.mean() for tsd in tree_size_data_dec]

# Boxplot for computation time and tree size by agent count (decentral)
plt.figure(figsize=(14, 8))
plt.subplot(1, 2, 1)
plt.boxplot(time_data_dec, labels=labels_dec)
plt.title('Decentral planner box plot of computation time by number of agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Computation Time (seconds)')
plt.legend(["Computation Time"], loc="upper right")

plt.subplot(1, 2, 2)
plt.boxplot(tree_size_data_dec, labels=labels_dec)
plt.title('Decentral planner box plot of tree size by number of agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Tree Size (total waypoints)')
plt.legend(["Tree Size"], loc="upper right")

plt.suptitle('Decentral planner box plot of computation time and tree size by number of agents')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('decentral_planner_combined_boxplot.png')
plt.show()

print('Combined boxplot for decentral planner saved as decentral_planner_combined_boxplot.png.')
plt.show()

# Plot average computation time vs. number of agents (decentral)
plt.figure(figsize=(8,6))
plt.plot(agent_counts_dec[:len(avg_times_dec)], avg_times_dec, marker='o')
plt.title('Decentral: Average Computation Time vs. Number of Agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Average Computation Time (seconds)')
plt.grid(True)
plt.savefig('decentral_avg_time_vs_agents.png')
plt.show()

# Plot average tree size vs. number of agents (decentral)
plt.figure(figsize=(8,6))
plt.plot(agent_counts_dec[:len(avg_tree_sizes_dec)], avg_tree_sizes_dec, marker='o')
plt.title('Decentral: Average Tree Size vs. Number of Agents')
plt.xlabel('Number of Agents (m)')
plt.ylabel('Average Tree Size (total waypoints)')
plt.grid(True)
plt.savefig('decentral_avg_tree_size_vs_agents.png')
plt.show()

print('Boxplots and summary plots saved for all agent counts (central and decentral).')
