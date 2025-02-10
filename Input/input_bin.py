import numpy as np

# Carica il file di input .npy
npy_file = ("subset_input.npy")
#npy_file = ("total_batch_input.npy")

data = np.load(npy_file)

# Converte i dati in float32 (se non lo sono già)
data_float32 = data.astype(np.float32)

# Salva i dati in formato binario
binary_file = "input.bin"
data_float32.tofile(binary_file)
