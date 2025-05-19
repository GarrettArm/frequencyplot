package main

import (
	"fmt"
	"sync"

	"github.com/gordonklaus/portaudio"
)

// MicrophoneGenerator handles microphone input
type MicrophoneGenerator struct {
	stream *portaudio.Stream
	done   chan struct{}
	mu     sync.RWMutex
}

func NewMicrophoneGenerator() (*MicrophoneGenerator, error) {
	g := &MicrophoneGenerator{
		done: make(chan struct{}),
	}
	
	stream, err := portaudio.OpenDefaultStream(1, 0, float64(sampleRate), bufferSize, g)
	if err != nil {
		return nil, fmt.Errorf("error opening stream: %v", err)
	}
	
	if err := stream.Start(); err != nil {
		stream.Close()
		return nil, fmt.Errorf("error starting stream: %v", err)
	}
	
	g.stream = stream
	return g, nil
}

func (g *MicrophoneGenerator) Generate() []float32 {
	g.mu.RLock()
	defer g.mu.RUnlock()
	
	if g.stream == nil {
		return make([]float32, bufferSize) // Return silent buffer if closed
	}
	
	buffer := make([]float64, bufferSize)
	err := g.stream.Read()
	if err != nil {
		return make([]float32, bufferSize) // Return silent buffer on error
	}
	
	select {
	case <-g.done:
		return make([]float32, bufferSize)
	default:
		// Convert float64 to float32
		result := make([]float32, len(buffer))
		for i, v := range buffer {
			result[i] = float32(v)
		}
		return result
	}
}

func (g *MicrophoneGenerator) Close() error {
	g.mu.Lock()
	defer g.mu.Unlock()
	
	if g.stream == nil {
		return nil
	}
	
	close(g.done)
	
	err := g.stream.Stop()
	if err != nil {
		return fmt.Errorf("error stopping stream: %v", err)
	}
	
	err = g.stream.Close()
	if err != nil {
		return fmt.Errorf("error closing stream: %v", err)
	}
	
	g.stream = nil
	return nil
}