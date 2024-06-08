import cv2, time, os, sys
import numpy as np
import ipywidgets.widgets as widgets

import atexit
isnotebook = True

IDENTIFIER="_models"

mnist_train_x=None
mnist_train_y=None
mnist_test_x=None
mnist_test_y=None


class Linear_Regression:
    learning_rate=1e-2
    print_every=10
    global_step=0

    _X_data=None
    _Y_data=None
    bias=None
    weight=None

    hypothesis=None

    optimizer=None

    ckpt_name="linear_regression" + IDENTIFIER
    restore=None

    _cb_fit=None

    @property
    def X_data(self):
        return self._X_data

    @X_data.setter
    def X_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 2:
            arr=np.expand_dims(arr, -1)

        self._X_data=arr

    @property
    def Y_data(self):
        return self._Y_data

    @Y_data.setter
    def Y_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 2:
            arr=np.expand_dims(arr, -1)

        self._Y_data=arr

    def on_epoch_end(self, epoch, logs):
        if (epoch + 1) % self.print_every == 0:
                print(self.global_step + 1, "step loss:", logs['loss'])
        self.global_step+=1

    def __init__(self, restore=False, ckpt_name=ckpt_name):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore = restore

        self._cb_fit.on_epoch_end=self.on_epoch_end

        self.hypothesis = Sequential()
        
        self.hypothesis.add(layers.Dense(1, input_shape=[1]))

        self.hypothesis.compile(optimizer=self.optimizer, loss='mse')

        if restore:
            if self.load(self.ckpt_name) is None:
                print("Create a new model.")

    def load(self, path=None):
        if path is None : path=self.ckpt_name

        if os.path.isfile(path+".index"):
            ckpt=path
            self.hypothesis.load_weights(ckpt)
            if ".ckpt-" in ckpt :
                self.global_step=int(ckpt.split(".ckpt-")[-1])
            else:
                print("[Warning] Can't read step log.")
                self.global_step=0

        elif os.path.isfile(path):
            exname = path.split(".")[-1]
            if exname == "index" or "data-" in exname :
                ckpt=path[:len(path)-(len(exname)+1)]
                self.hypothesis.load_weights(ckpt)
                if "model.ckpt-" in ckpt :
                    self.global_step=int(ckpt.split("model.ckpt-")[1])
                else:
                    print("[Warning] Can't read step log.")
                    self.global_step=0
            else:
                print("[Error] Can't find a model in '"+path+"'.")
                return

        elif os.path.isdir(path):
            if os.path.isfile(path+"/checkpoint"):
                ckpt=tf.train.latest_checkpoint(path)
                self.hypothesis.load_weights(ckpt)
                if "model.ckpt-" in ckpt :
                    self.global_step=int(ckpt.split("model.ckpt-")[1])
                else:
                    print("[Warning] Can't read step log.")
                    self.global_step=0
            else:
                print("[Error] Doesn't exist a checkpoint file in '"+path+"'.")
                return

        else:
            print("[Error] Can't find a model in '"+path+"'.")
            return

        self.ckpt_name = path
        return "Loaded successfully."

    def save(self, path=None):
        if path is None : path=self.ckpt_name

        if self.hypothesis is not None: 
            self.hypothesis.save_weights(path+"/model.ckpt-"+str(self.global_step))

    def train(self, times=100, print_every=10):
        self.print_every=print_every

        dtime=time.time()
        
        if self.X_data is not None and self.Y_data is not None :
            X_data = tf.keras.preprocessing.sequence.pad_sequences(self.X_data).astype(np.float32)
            Y_data = tf.keras.preprocessing.sequence.pad_sequences(self.Y_data).astype(np.float32)
            self.hypothesis.fit(X_data, Y_data, epochs=times, verbose=0, callbacks=[self._cb_fit], use_multiprocessing=True)
        elif self.X_data is None :
            print("Please input a data to X_data.")
            return
        elif self.Y_data is None :
            print("Please input a data to Y_data.")
            return

        print("Training is done.\nTime spent:", round(time.time()-dtime,1), "s\nTraining speed:", round(times/(time.time()-dtime),1), "step/s")

        self.save()

    def run(self,inputs=None):
        if inputs is None:
            inputs=self.X_data
            print("run with : ",inputs)

        inputs=np.array(inputs, dtype=np.float32)
        while len(inputs.shape) < 2:
            inputs=np.expand_dims(inputs, -1)

        if self.hypothesis is not None and inputs is not None :
            if isnotebook:
                return self.hypothesis.predict(inputs, use_multiprocessing=True)
            else:
                ret = self.hypothesis.predict(inputs, use_multiprocessing=True)
                print(ret)
                return ret


class Logistic_Regression(Linear_Regression):
    ckpt_name="logistic_regression" + IDENTIFIER
    learning_rate=1e-1
    optimizer=None

    def __init__(self, input_size=1, restore=False, ckpt_name=ckpt_name):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name
        self.restore = restore
        self._cb_fit.on_epoch_end=self.on_epoch_end

        self.hypothesis = Sequential()

        self.hypothesis.add(layers.Dense(1, input_shape=[input_size], activation='sigmoid'))

        self.hypothesis.compile(optimizer=self.optimizer, loss='binary_crossentropy')

        if restore:
            if self.load(self.ckpt_name) is None:
                print("Create a new model.")

    def loss_function(self, y_true, y_pred):
        y_true=tf.cast(y_true,tf.float32)
        y_pred=tf.cast(y_pred,tf.float32)
        return -tf.reduce_mean(y_true * tf.math.log(y_pred) + (1 - y_true) * (tf.math.log(1 - y_pred)),-1)

class Perceptron:
    learning_rate=1e-2
    print_every=10
    global_step=0

    softmax=True

    _X_data=None
    _Y_data=None
    bias=None
    weight=None
    
    model=None
    _mnist_loaded=False

    optimizer=None

    ckpt_name="perceptron" + IDENTIFIER
    restore=None

    _cb_fit=None

    @property
    def X_data(self):
        return self._X_data

    @X_data.setter
    def X_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 2:
            arr=np.expand_dims(arr, -1)

        self._X_data=arr

    @property
    def Y_data(self):
        return self._Y_data

    @Y_data.setter
    def Y_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 2:
            arr=np.expand_dims(arr, -1)

        self._Y_data=arr

    def on_epoch_end(self, epoch, logs):
        if (epoch + 1) % self.print_every == 0:
                print(self.global_step + 1, "step loss:", logs['loss'])
        self.global_step+=1

    def __init__(self, input_size, output_size=1, restore=False, ckpt_name=ckpt_name, softmax=True, activation_function=None):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_epoch_end=self.on_epoch_end

        self.model = Sequential()

        if self.softmax :
            if output_size > 1 :
                self.model.add(layers.Dense(output_size,input_shape=[input_size], activation='softmax'))

                self.model.compile(optimizer=self.optimizer, loss='categorical_crossentropy')
            else :
                self.model.add(layers.Dense(output_size,input_shape=[input_size], activation='sigmoid'))
                
                self.model.compile(optimizer=self.optimizer, loss='binary_crossentropy')
        else :
            if activation_function == None:
                self.model.add(layers.Dense(output_size,input_shape=[input_size]))
                
                self.model.compile(optimizer=self.optimizer, loss='mse')
            else:
                self.model.add(layers.Dense(output_size,input_shape=[input_size], activation=activation_function))
                
                self.model.compile(optimizer=self.optimizer, loss='mse')

        if restore:
            if self.load(self.ckpt_name) is None:
                print("Create a new model.")


    def load(self, path=None):
        if path is None : path=self.ckpt_name

        if os.path.isfile(path+".index"):
            ckpt=path
            self.model.load_weights(ckpt)
            if ".ckpt-" in ckpt :
                self.global_step=int(ckpt.split(".ckpt-")[-1])
            else:
                print("[Warning] Can't read step log.")
                self.global_step=0

        elif os.path.isfile(path):
            exname = path.split(".")[-1]
            if exname == "index" or "data-" in exname :
                ckpt=path[:len(path)-(len(exname)+1)]
                self.model.load_weights(ckpt)
                if "model.ckpt-" in ckpt :
                    self.global_step=int(ckpt.split("model.ckpt-")[1])
                else:
                    print("[Warning] Can't read step log.")
                    self.global_step=0
            else:
                print("[Error] Can't find a model in '"+path+"'.")
                return

        elif os.path.isdir(path):
            if os.path.isfile(path+"/checkpoint"):
                ckpt=tf.train.latest_checkpoint(path)
                self.model.load_weights(ckpt)
                if "model.ckpt-" in ckpt :
                    self.global_step=int(ckpt.split("model.ckpt-")[1])
                else:
                    print("[Warning] Can't read step log.")
                    self.global_step=0
            else:
                print("[Error] Doesn't exist a checkpoint file in '"+path+"'.")
                return

        else:
            print("[Error] Can't find a model in '"+path+"'.")
            return

        self.ckpt_name = path
        return "Loaded successfully."

    def save(self, path=None):
        if path is None : path=self.ckpt_name
        
        if self.model is not None: 
            self.model.save_weights(path+"/model.ckpt-"+str(self.global_step))

    def train(self, times=100, print_every=10):
        self.print_every=print_every

        dtime=time.time()
        
        if self.X_data is not None and self.Y_data is not None :
            X_data = tf.keras.preprocessing.sequence.pad_sequences(self.X_data).astype(np.float32)
            Y_data = tf.keras.preprocessing.sequence.pad_sequences(self.Y_data).astype(np.float32)
            self.model.fit(X_data, Y_data, epochs=times, verbose=0, callbacks=[self._cb_fit], use_multiprocessing=True)
        elif self.X_data is None :
            print("Please input a data to X_data.")
            return
        elif self.Y_data is None :
            print("Please input a data to Y_data.")
            return

        print("Training is done.\nTime spent:", round(time.time()-dtime,1), "s\nTraining speed:", round(times/(time.time()-dtime),1), "step/s")

        self.save()

    def run(self,inputs=None):
        if inputs is None:
            inputs=self.X_data
            print("run with : ",inputs)

        inputs=np.array(inputs, dtype=np.float32)
        while len(inputs.shape) < 2:
            inputs=np.expand_dims(inputs, -1)

        if self.model is not None and inputs is not None :
            return self.model.predict(inputs, use_multiprocessing=True)

class ANN(Perceptron):
    layer=None
    ckpt_name="ANN" + IDENTIFIER

    def __init__(self, input_size, hidden_size=10, output_size=1, restore=False, ckpt_name=ckpt_name, softmax=True):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, Input, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_epoch_end=self.on_epoch_end

        self.model = Sequential()

        if self.softmax :
            if output_size > 1 :
                self.model.add(layers.Dense(hidden_size,input_shape=[input_size]))
                self.model.add(layers.Dense(output_size, activation='softmax'))

                self.model.compile(optimizer=self.optimizer, loss='categorical_crossentropy')
            else :
                self.model.add(layers.Dense(hidden_size,input_shape=[input_size]))
                self.model.add(layers.Dense(output_size, activation='sigmoid'))
                
                self.model.compile(optimizer=self.optimizer, loss='binary_crossentropy')
        else :
            self.model.add(layers.Dense(hidden_size,input_shape=[input_size]))
            self.model.add(layers.Dense(output_size))
            
            self.model.compile(optimizer=self.optimizer, loss='mse')

        if restore:
            if self.load(self.ckpt_name) is None:
                print("Create a new model.")

class DNN(ANN):
    learning_rate=1e-2
    level=0
    ckpt_name="DNN" + IDENTIFIER
    optimizer=None

    def __init__(self, input_size, hidden_size=10, output_size=1, layer_level=3, restore=False, ckpt_name=ckpt_name, softmax=True):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_epoch_end=self.on_epoch_end

        if layer_level < 1:
            print("Please set a layer level at least 1.")
            del self
        else:
            self.model = Sequential()

            self.model.add(layers.InputLayer(input_shape=(input_size,)))
            
            for _ in range(layer_level):
                self.model.add(layers.Dense(hidden_size))

            if self.softmax :
                if output_size > 1 :
                    self.model.add(layers.Dense(output_size, activation='softmax'))

                    self.model.compile(optimizer=self.optimizer, loss='categorical_crossentropy')
                else :
                    self.model.add(layers.Dense(output_size, activation='sigmoid'))
                    
                    self.model.compile(optimizer=self.optimizer, loss='binary_crossentropy')
            else :
                self.model.add(layers.Dense(output_size))
                
                self.model.compile(optimizer=self.optimizer, loss='mse')

            if restore:
                if self.load(self.ckpt_name) is None:
                    print("Create a new model.")

class CNN(DNN):
    ckpt_name="CNN" + IDENTIFIER
    _input_level=1
    mnist_train_x=None
    mnist_train_y=None
    mnist_test_x=None
    mnist_test_y=None
    _X=None
    _Y=None
    _pre_batch_pos=0
    gray=True

    @property
    def X_data(self):
        return self._X_data

    @X_data.setter
    def X_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 3:
            if len(arr.shape) < 2 :
                arr=np.expand_dims(arr, -1)
            else :
                arr=np.expand_dims(arr, 0)

        self._X_data=arr

    @property
    def Y_data(self):
        return self._Y_data

    @Y_data.setter
    def Y_data(self, arg):
        arr=np.array(arg)
        while len(arr.shape) < 2 :
            arr=np.expand_dims(arr, -1)

        self._Y_data=arr

    def __init__(self, input_size=[28,28], input_level=1, kernel_size=[3,3], kernel_count=32, strides=[1,1], hidden_size=128, output_size=1, conv_level=2, layer_level=1, restore=False, ckpt_name=ckpt_name, softmax=True):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self._ipsize=input_size
        self._opsize=output_size
        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_epoch_end=self.on_epoch_end

        input_shape=[input_size[0], input_size[1], input_level]

        if input_level == 3 :
            self.gray=False
        
        if layer_level < 1 :
            print("Please set a Fully-connected layer level at least 1.")
            del self
        elif conv_level < 1 :
            print("Please set a Convolutional layer level at least 1.")
            del self
        else:
            self.model = Sequential()

            self.model.add(layers.InputLayer(input_shape=input_shape))
            
            for _ in range(conv_level):
                self.model.add(layers.Conv2D(filters=kernel_count, kernel_size=kernel_size, strides=strides, padding='SAME'))
                self.model.add(layers.MaxPool2D([2,2], [2,2], padding='SAME'))
                self.model.add(layers.Dropout(0.5))

            self.model.add(layers.Flatten())

            for _ in range(layer_level):
                self.model.add(layers.Dense(hidden_size))
                self.model.add(layers.Dropout(0.3))

            if self.softmax :
                if output_size > 1 :
                    self.model.add(layers.Dense(output_size, activation='softmax'))

                    self.model.compile(optimizer=self.optimizer, loss='categorical_crossentropy')
                else :
                    self.model.add(layers.Dense(output_size, activation='sigmoid'))
                    
                    self.model.compile(optimizer=self.optimizer, loss='binary_crossentropy')
            else :
                self.model.add(layers.Dense(output_size))
                
                self.model.compile(optimizer=self.optimizer, loss='mse')

            if restore:
                if self.load(self.ckpt_name) is None:
                    print("Create a new model.")

    def train(self, times=100, batch=500, print_every=10):
        self.print_every=print_every

        dtime=time.time()
        
        if self.X_data is not None and self.Y_data is not None :
            X_data = tf.keras.preprocessing.sequence.pad_sequences(self.X_data).astype(np.float32)
            Y_data = tf.keras.preprocessing.sequence.pad_sequences(self.Y_data).astype(np.float32)
            self.model.fit(X_data, Y_data, epochs=times, batch_size=batch, steps_per_epoch=1, verbose=0, callbacks=[self._cb_fit], use_multiprocessing=True)
        elif self.X_data is None :
            print("Please input a data to X_data.")
            return
        elif self.Y_data is None :
            print("Please input a data to Y_data.")
            return

        print("Training is done.\nTime spent:", round(time.time()-dtime,1), "s\nTraining speed:", round(times/(time.time()-dtime),1), "step/s")

        self.save()

    def run(self,inputs=None,show=True):
        if inputs is None:
            inputs=self.X_data
            print("run with : ",inputs)

        inputs=np.array(inputs, dtype=np.float32)
        while len(inputs.shape) < 3:
            if len(inputs.shape) < 2 :
                inputs=np.expand_dims(inputs, -1)
            else :
                inputs=np.expand_dims(inputs, 0)

        warned_shape=False

        for input in inputs:
            while len(tf.shape(input)) < 3:
                if not warned_shape:
                    print("[Warning] Inputs shape doesn't match. Automatically transformed to 4 Dimensions but may be occur errors or delay.")
                    warned_shape=True
                tf.expand_dims(input,0)

        if len(inputs)<5 and show:
            global plt
            import matplotlib.pyplot as plt
            for input in inputs:
                if self.gray:
                    plt.imshow(input.reshape(self._ipsize[0],self._ipsize[1]), cmap='gray', vmin=0, vmax=255)
                else:
                    input = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
                    plt.imshow(input.astype('uint8'))
                if isnotebook:
                    plt.show()
                else:
                    plt.savefig('cnn_temp.png', dpi=300)

        inputs=tf.cast(inputs,tf.float32)

        if self.model is not None and inputs is not None :
            return self.model.predict(inputs, use_multiprocessing=True)

    def load_MNIST(self):
        global mnist_train_x, mnist_train_y, mnist_test_x, mnist_test_y
        if mnist_train_x is None and mnist_train_y is None:
            (self.mnist_train_x,self.mnist_train_y), (self.mnist_test_x, self.mnist_test_y) = tf.keras.datasets.mnist.load_data()
            self.mnist_train_x=self.mnist_train_x.reshape(-1, 28, 28, 1)
            self.mnist_train_y=tf.one_hot(self.mnist_train_y, 10)
            self.mnist_test_x=self.mnist_test_x.reshape(-1, 28, 28, 1)
            self.mnist_test_y=tf.one_hot(self.mnist_test_y, 10)
            self.X_data=self.mnist_train_x
            self.Y_data=self.mnist_train_y
        else:
            self.mnist_train_x=mnist_train_x
            self.mnist_train_y=mnist_train_y
            self.mnist_test_x=mnist_test_x
            self.mnist_test_y=mnist_test_y
            self.mnist_train_x=self.mnist_train_x.reshape(-1, 28, 28, 1)
            self.mnist_train_y=tf.one_hot(self.mnist_train_y, 10)
            self.mnist_test_x=self.mnist_test_x.reshape(-1, 28, 28, 1)
            self.mnist_test_y=tf.one_hot(self.mnist_test_y, 10)
            self.X_data=self.mnist_train_x
            self.Y_data=self.mnist_train_y

    def show_img(self, input):
        global plt
        import matplotlib.pyplot as plt
        if self.gray:
            plt.imshow(input.reshape(self._ipsize[0],self._ipsize[1]), cmap='gray', vmin=0, vmax=255)
        else:
            plt.imshow(input.reshape(self._ipsize[0],self._ipsize[1]))
        plt.show()

class RNN(ANN):
    learning_rate=1e-2
    level=0
    ckpt_name="RNN" + IDENTIFIER
    optimizer=None

    # @property
    # def X_data(self):
    #     return self._X_data

    # @X_data.setter
    # def X_data(self, arg):
    #     arr=np.array(arg)
    #     size=arr.shape
    #     arr=arr.reshape((size[0], size[1], 1))
    #     self._X_data=arr

    # @property
    # def Y_data(self):
    #     return self._Y_data

    # @Y_data.setter
    # def Y_data(self, arg):
    #     arr=np.array(arg)
    #     self._Y_data=arr

    def __init__(self, input_size, hidden_size=64, output_size=1, layer_level=1, restore=False, ckpt_name=ckpt_name, softmax=False):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_epoch_end=self.on_epoch_end
    
        if layer_level < 1:
            print("Please set a layer level at least 1.")
            del self
        else:
            self.model = Sequential()

            self.model.add(layers.InputLayer(input_shape=(input_size,1)))
            
            self.model.add(layers.Conv1D(filters=32, kernel_size=3, strides=1, padding='same'))
            self.model.add(layers.MaxPooling1D(pool_size=2))
            #self.model.add(layers.Dropout(0.3))

            #self.model.add(layers.Conv1D(filters=16, kernel_size=11, strides=1, padding='valid', activation='relu'))
            #self.model.add(layers.MaxPooling1D(pool_size=3))
            #self.model.add(layers.Dropout(0.3))

            #self.model.add(layers.Conv1D(filters=32, kernel_size=9, strides=1, padding='valid', activation='relu'))
            #self.model.add(layers.MaxPooling1D(pool_size=3))
            #self.model.add(layers.Dropout(0.3))

            for _ in range(layer_level-1):
                #self.model.add(layers.Bidirectional(layers.GRU(hidden_size, return_sequences = True), merge_mode='sum'))
                self.model.add(layers.LSTM(hidden_size, return_sequences = True))

            #self.model.add(layers.Bidirectional(layers.GRU(hidden_size, return_sequences = False), merge_mode='sum'))
            self.model.add(layers.LSTM(hidden_size, return_sequences = False))

            if self.softmax :
                if output_size > 1 :
                    self.model.add(layers.Dense(output_size, activation='softmax'))

                    self.model.compile(optimizer=self.optimizer, loss='categorical_crossentropy')
                else :
                    self.model.add(layers.Dense(output_size, activation='sigmoid'))
                    
                    self.model.compile(optimizer=self.optimizer, loss='binary_crossentropy')
            else :
                self.model.add(layers.Dense(output_size))
                
                self.model.compile(optimizer=self.optimizer, loss='mse')

            if restore:
                if self.load(self.ckpt_name) is None:
                    print("Create a new model.")

    def train(self, times=100, batch=50, print_every=10):
        self.print_every=print_every

        dtime=time.time()
        
        if self.X_data is not None and self.Y_data is not None :
            # print(self.X_data.shape, self.Y_data.shape)
            X_data = tf.keras.preprocessing.sequence.pad_sequences(self.X_data).astype(np.float32)
            Y_data = tf.keras.preprocessing.sequence.pad_sequences(self.Y_data).astype(np.float32)
            self.model.fit(X_data, Y_data, epochs=times, verbose=0, callbacks=[self._cb_fit], use_multiprocessing=True)
        elif self.X_data is None :
            print("Please input a data to X_data.")
            return
        elif self.Y_data is None :
            print("Please input a data to Y_data.")
            return

        print("Training is done.\nTime spent:", round(time.time()-dtime,1), "s\nTraining speed:", round(times/(time.time()-dtime),1), "step/s")

        self.save()

    def run(self,inputs=None):
        if inputs is None:
            inputs=self.X_data
            print("run with : ",inputs)

        inputs=np.array(inputs, dtype=np.float32)
        size=inputs.shape
        inputs=inputs.reshape((size[0], size[1], 1))

        if self.model is not None and inputs is not None :
            return self.model.predict(inputs, use_multiprocessing=True)

class DQN(DNN):
    learning_rate=1e-2
    ckpt_name="DQN" + IDENTIFIER
    rewards=None
    prob=None
    low_limit=1
    low_limit_count=0
    
    def on_train_end(self, logs):
        self.global_step+=1

    def __init__(self, state_size, hidden_size=5, output_size=1, layer_level=1, restore=False, ckpt_name=ckpt_name, softmax=True):
        global tf, layers, Sequential, optimizers, callbacks, losses, Adam, load_model, model_from_json, Model, ctivation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add, K
        import tensorflow as tf
        from tensorflow.keras import layers, Sequential, optimizers, callbacks, losses
        from tensorflow.keras.optimizers import Adam
        from tensorflow.keras.models import load_model, model_from_json
        from tensorflow.keras.models import Model
        from tensorflow.keras.layers import Activation, BatchNormalization, Concatenate, Conv2D, Dense, Dropout, GlobalAveragePooling2D, InputLayer, Lambda, MaxPooling2D, add
        from tensorflow.keras import backend as K

        self.optimizer=Adam(lr=self.learning_rate)
        self._cb_fit=callbacks.Callback()

        self.ckpt_name = ckpt_name

        self.restore=restore
        self.softmax=softmax

        self._cb_fit.on_train_end=self.on_train_end

        self.state_size=state_size

        if layer_level < 1:
            print("Please set a layer level at least 1.")
            del self
        else:
            self.model = Sequential()

            self.model.add(layers.InputLayer(input_shape=(state_size,)))
            
            for _ in range(layer_level):
                self.model.add(layers.Dense(hidden_size, kernel_initializer='he_uniform'))

            if self.softmax :
                if output_size > 1 :
                    self.model.add(layers.Dense(output_size, activation='softmax'))

                    self.model.compile(optimizer=self.optimizer, loss=self._catcn_loss)
                else :
                    self.model.add(layers.Dense(output_size, activation='sigmoid'))
                    
                    self.model.compile(optimizer=self.optimizer, loss=self._bincn_loss)
            else :
                self.model.add(layers.Dense(output_size))
                
                self.model.compile(optimizer=self.optimizer, loss=self._mse_loss)

            if restore:
                if self.load(self.ckpt_name) is None:
                    print("Create a new model.")

    def _loss_process(self, loss):
        try : return tf.math.pow(1/tf.math.sqrt(tf.reduce_mean(tf.expand_dims(loss,-1) * self.rewards, -1))*10, 2.)
        except : return loss

    def _bincn_loss(self, y_true, y_pred):
        return self._loss_process(losses.binary_crossentropy(y_true, y_pred))

    def _catcn_loss(self, y_true, y_pred):
        return self._loss_process(losses.categorical_crossentropy(y_true, y_pred))

    def _mse_loss(self, y_true, y_pred):
        return self._loss_process(losses.mse(y_true, y_pred))

    def train(self, states, rewards, actions, times=1, reward_std="time"):
        if reward_std=="time":
            self.rewards=self.process_rewards(rewards)
            states=np.array(states)
            rewards=np.array(rewards)
            actions=tf.cast(actions,tf.float32)

            self.low_limit=max(len(rewards),self.low_limit)#(self.low_limit_count/(self.low_limit_count+1))*self.low_limit+(len(rewards)*1.2)/(self.low_limit_count+1)
            #self.low_limit_count+=1

            hist=self.model.fit(states, actions, epochs=times, verbose=0, callbacks=[self._cb_fit], use_multiprocessing=True)
        
        return tf.reduce_mean(hist.history['loss']).numpy()

    def run(self,inputs=None, boolean=True):
        if inputs is None:
            inputs=self.X_data
            print("run with : ",inputs)
        
        inputs=np.array(inputs, dtype=np.float32)
        while len(inputs.shape) < 2:
            inputs=np.expand_dims(inputs, -1)

        if self.model is not None and inputs is not None :
            if boolean:
                pred=self.model.predict(inputs, use_multiprocessing=True)
                return np.bool(pred>=0.5)
            else:
                return self.model.predict(inputs, use_multiprocessing=True)
                
    def process_rewards(self, r):
        dr = np.zeros_like(r)

        limit=round(self.low_limit*0.7)
        
        tmp=0
        cnt=0
        for i in range(len(r)-limit,len(r)):
            if i>=0:
                tmp+=r[i]
                cnt+=1
        
        dr[-1]=tmp/cnt*limit

        for i in reversed(range(len(r)-limit,len(r)-1)):
            if i>=0:
                dr[i]=dr[i+1]-r[i+1]
            
        for i in reversed(range(len(r)-limit,len(r))):
            if i>=0:
                dr[i]=1/dr[i]
            
        for i in reversed(range(0,len(r)-limit)):
            if i>=0:
                dr[i]=dr[i+1]+r[i+1]

        #dr[-1] = r[-1]
        #for t in reversed(range(0, len(r)-1)):
        #    dr[t] = dr[t+1] + r[t]
        
        return dr#np.power(dr,2)
