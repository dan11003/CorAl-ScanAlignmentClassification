# Statistics output for experiments
import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Read provided statistics file
bag_location = os.getenv('BAG_LOCATION')
directory = bag_location + '/CoralRadarEval/'
parser = argparse.ArgumentParser()
parser.add_argument("--filename", "-f", help="csv or text file containing statistics", default="")
args = parser.parse_args()

# Import statistics file
stats_file = directory + args.filename
data = pd.read_csv(stats_file)

# Directory to save figures
filename = args.filename
save_dir = directory + filename.replace(".txt","")
if not os.path.exists(save_dir) :
    os.mkdir(save_dir)

#  -- Plot 1 (accuracy by method) -- #

# Plot style (seaborn)
sns.set_theme(style="ticks", color_codes=True)
sns.set(style="ticks")
colors = ["#E59866", "#D35400", "#873600", "#85C1E9", "#3498DB", "#2874A6"]   #Daniel
#colors = ["#E59866","#2874A6", "#D35400", "#85C1E9", "#3498DB",  "#873600"]  #Mine
data = data.rename(columns={'range error': 'Range error (m)'})
data = data.rename(columns={'accuracy': 'Accuracy'})
data = data.rename(columns={'method': 'Method'})
data = data.rename(columns={'auc': 'AuC'})

pal = sns.color_palette(colors)
mar = ['o','o','o','s','s','s']
lin = ['-','--',':','-','--',':']

# Plot data
#data['method'] = data.apply(lambda row:  row['method'].replace('method', ''), axis=1)

#Lineplot (commented)
#sns_plot = sns.pointplot(x = 'Range error (m)', y = 'Accuracy', data = data, hue = 'Method', dodge = False, palette=pal)
#sns_plot.locator_params(axis='x', nbins=5)

# Uncertainty plot
#sns_plot = sns.catplot(x='Range error (m)', y='Accuracy', hue='Method', kind="box", dodge=False, data=data, legend_out=True) # Old
sns_plot = sns.lineplot(x="Range error (m)", y="Accuracy", hue="Method", style="Method", data=data, markers=['o','o','o'], dashes=False) # Current
plt.ylim(0.45,1)
plt.xlim(0.05,0.95)

plt.grid()
plt.show()
#fig = sns_plot.get_figure()
#fig.savefig(save_dir + '/accuracyByMethod.pdf', format='pdf')


# Print parameters for 'best' configuration for each method
data_p2p = data[data['Method'] == 'P2P']
data_p2d = data[data['Method'] == 'P2D']
data_p2l = data[data['Method'] == 'P2L']

print('Best configurations for all methods: ') 
print(data_p2p[data_p2p.Accuracy == data_p2p.Accuracy.max()]) ; print(' ')
print(data_p2d[data_p2d.Accuracy == data_p2d.Accuracy.max()]) ; print(' ')
print(data_p2l[data_p2l.Accuracy == data_p2l.Accuracy.max()]) ; print(' ')

# Box plot (accuracies and AuC for all methods)
sns_plot_2 = sns.boxplot(x=data['Method'], y=data['Accuracy'], width=0.3)
plt.show()
#fig = sns_plot_2.get_figure()
#fig.savefig(save_dir + '/accuracyBoxplot.pdf', format='pdf')

sns_plot_3 = sns.boxplot(x=data['Method'], y=data['AuC'], width=0.3)
plt.show()
#fig = sns_plot_3.get_figure()
#fig.savefig(save_dir + '/aucBoxplot.pdf', format='pdf')




#  --- OLD CODE (standalone matplot lib, no seaborn) --- #

# Filter data by method
# data_p2p = data[data['method'] == 'P2P']
# data_p2d = data[data['method'] == 'P2D']
# data_p2l = data[data['method'] == 'P2L']

# # Get data
# x_p2p = data_p2p['range error'] ; y_p2p = data_p2p['accuracy']
# x_p2d = data_p2d['range error'] ; y_p2d = data_p2d['accuracy']
# x_p2l = data_p2l['range error'] ; y_p2l = data_p2l['accuracy']

# # Plot data
# plt.plot(x_p2p,y_p2p,marker='o',label='P2P')
# plt.plot(x_p2d,y_p2d,marker='o',label='P2D')
# plt.plot(x_p2l,y_p2l,marker='o',label='P2L')
# plt.legend(loc="upper left")
# plt.xlabel('Range error (m)')
# plt.ylabel('Classification accuracy')
# plt.savefig(save_dir + '/accuracyByMethod.pdf')
# plt.show()

# # -- Plot 2 (boxplot by method) -- #

# accuracies = [y_p2p, y_p2d, y_p2l]
# plt.boxplot(accuracies)
# plt.xticks([1, 2, 3], ['P2P','P2D','P2L'])
# plt.ylabel('Classification accuracy')
# plt.savefig(save_dir + '/boxplotByMethod.pdf')
# plt.show()