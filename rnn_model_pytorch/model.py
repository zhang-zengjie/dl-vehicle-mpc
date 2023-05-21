import torch
import torch.nn as nn


class PredictionNet(nn.Module):

    def __init__(self, out_len, batch_size):
        super(PredictionNet, self).__init__()

        torch.manual_seed(0)

        self.input_dim = 2  # x,y-position --> 2 dimensions
        self.input_embedding_size = 32
        self.encoder_size = 64
        self.num_layers_enc = 2
        self.latent_emb_size = 64
        self.num_layers_dec = 2
        self.decoder_size = 128
        self.out_length = out_len
        self.bool_norm = True  # batch normalization
        self.dropout = 0.2  # dropout layer
        self.output_dim = 2
        self.batch_size = batch_size

        ################################
        # Define Network#
        ################################
        # input embedding
        self.ip_emb = torch.nn.Linear(
            in_features=self.input_dim, out_features=self.input_embedding_size
        )

        # encoder rnn
        self.encoder_rnn = torch.nn.GRU(
            input_size=self.input_embedding_size,
            hidden_size=self.encoder_size,
            num_layers=self.num_layers_enc,
            dropout=self.dropout,
        )

        # expand latent space
        self.latent_emb = torch.nn.Linear(
            in_features=self.encoder_size, out_features=self.latent_emb_size
        )

        # decoder rnn
        self.decoder_rnn = torch.nn.GRU(
            input_size=self.latent_emb_size,
            hidden_size=self.decoder_size,
            num_layers=self.num_layers_dec,
            dropout=self.dropout,
        )

        # output layer
        self.output_layer = torch.nn.Linear(
            in_features=self.decoder_size, out_features=self.output_dim
        )

        self.enc_normalize = torch.nn.BatchNorm1d(self.batch_size)
        self.dec_normalize = torch.nn.BatchNorm1d(self.batch_size)

        self.normalize = self.bool_norm

        self.ReLU = torch.nn.ReLU()

    def forward(self, hist):
        # pass input through fully connected + ReLU-activation
        fully_output = self.ip_emb(hist.permute(2, 0, 1))
        fully_output = self.ReLU(fully_output)

        # .. then feed into encoder-RNN
        _, hidden_enc = self.encoder_rnn(fully_output)

        if self.normalize:
            hidden_enc = torch.nn.BatchNorm1d(hidden_enc.shape[1])(hidden_enc)

        # .. next: pass extracted features into decoder
        hidden_enc = hidden_enc[-1:, :, :]  # use only last hidden dimension

        # embed latent space
        dec_in = self.ReLU(self.latent_emb(hidden_enc))

        # .. map to output rnn
        dec_in = dec_in.repeat(self.out_length, 1, 1)

        # decode latent space
        dec_out, _ = self.decoder_rnn(dec_in)
        if self.normalize:
            dec_out = torch.nn.BatchNorm1d(dec_out.shape[1])(dec_out)

        # .. linear layer for dimension reduction
        pred_out = self.output_layer(dec_out)

        return pred_out.permute(1, 2, 0)


#Train Latent ODE Network with Irregularly Sampled Time-Series Data