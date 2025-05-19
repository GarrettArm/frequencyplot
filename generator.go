package main

// AudioGenerator interface defines methods for audio generation
type AudioGenerator interface {
	Generate() []float32
	Close() error
}