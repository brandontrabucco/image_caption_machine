import tensorflow as tf
import numpy as np
import os

class Machine(object):

    def __init__(self):
        self.ip_address = "192.168.1.1:8080"
        self.keyphrase = "14151337"

    def run_machine(self, image):

        g = tf.Graph()
        with g.as_default():

            input_image = tf.placeholder(
                tf.float32,
                shape=[1, 299, 299, 3],
                name="input_image")

            conv_filter = tf.get_variable(
                "conv_filter",
                shape=[5, 5, 3, 100],
                initializer=tf.truncated_normal_initializer())
            conv_bias = tf.get_variable(
                "conv_bias",
                shape=[1, 1, 1, 100],
                initializer=tf.constant_initializer(1.0))
            hidden_layer_one = tf.nn.conv2d(
                input_image,
                conv_filter,
                [1, 2, 2, 1],
                "VALID",
                name="hidden_layer_one") + conv_bias

            context_vector = tf.reduce_mean(
                hidden_layer_one,
                [1, 2],
                name="context_vector")

            lstm_cell = tf.contrib.rnn.LSTMCell(100)
            initial_state = lstm_cell.zero_state(1, tf.float32)
            sequence_outputs, state = tf.nn.dynamic_rnn(
                lstm_cell,
                tf.tile(tf.reshape(context_vector, [1, 1, 100]), [1, 6, 1]),
                initial_state=initial_state)

            fully_connected_weights = tf.get_variable(
                "fully_connected_weights",
                shape=[100, 1000],
                initializer=tf.truncated_normal_initializer())
            fully_connected_biases = tf.get_variable(
                "fully_connected_biases",
                shape=[1, 1, 1000],
                initializer=tf.constant_initializer(1.0))
            hidden_layer_two = tf.tensordot(
                sequence_outputs, fully_connected_weights, 1) + fully_connected_biases

            probabilities = tf.nn.softmax(
                hidden_layer_two, axis=-1, name="caption_output")
            caption_indices = tf.argmax(
                probabilities, axis=-1, name="caption_indices")
            init_op = tf.global_variables_initializer()

        g.finalize()
        session = tf.Session(graph=g)
        session.run(init_op)
        indices = session.run(
            caption_indices,
            feed_dict={
                input_image: image[np.newaxis, :, :, :]})
        indices = np.squeeze(indices)

        rel_dir = os.path.dirname(__file__)
        ext_dir = "data/word.vocab"
        joined_dir = os.path.join(rel_dir, ext_dir)
        with open(joined_dir, "r") as f:
            vocab = f.readlines()
        vocab = [v.strip() for v in vocab]
        caption_string = ""
        for i in indices:
            caption_string += vocab[i] + " "
        return caption_string
