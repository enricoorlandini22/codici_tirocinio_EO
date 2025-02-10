import numpy as np

# Carica il file originale
file_path_in = "total_batch_input.npy"
file_path_out = "total_batch_output.npy"
data_in = np.load(file_path_in)
data_out = np.load(file_path_out)

# Verifica le dimensioni
print("Shape originale (in):", data_in.shape)
print("Shape originale (out):", data_out.shape)

# Seleziona le prime 32 e le ultime 32 righe
subset_data_in = np.vstack([data_in[:2048], data_in[-2048:]])  # Combina le prime e ultime righe
subset_data_out = np.concatenate([data_out[:2048], data_out[-2048:]])

# Salva il nuovo array in un file .npy
output_in_file_path = "subset_input.npy"
output_out_file_path = "subset_output.npy"
np.save(output_in_file_path, subset_data_in)
np.save(output_out_file_path, subset_data_out)

# Verifica il nuovo file
print("Shape del nuovo file (in):", subset_data_in.shape)
print("Shape del nuovo file (out):", subset_data_out.shape)




