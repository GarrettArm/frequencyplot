package main

import (
	"fmt"
	"sync"

	"github.com/gordonklaus/portaudio"
)

// StreamProcessor handles continuous audio streaming
type StreamProcessor struct {
    stream     *portaudio.Stream
    generator  AudioGenerator
    isRunning  bool
    mu         sync.Mutex
}

func NewStreamProcessor(generator AudioGenerator) (*StreamProcessor, error) {
    sp := &StreamProcessor{
        generator: generator,
    }
    
    stream, err := portaudio.OpenDefaultStream(0, 1, float64(sampleRate), bufferSize, sp.processAudio)
    if err != nil {
        return nil, fmt.Errorf("error opening stream: %v", err)
    }
    sp.stream = stream
    return sp, nil
}

func (sp *StreamProcessor) processAudio(_, out []float32) {
    sp.mu.Lock()
    if !sp.isRunning {
        sp.mu.Unlock()
        return
    }
    sp.mu.Unlock()
    
    samples := sp.generator.Generate()
    copy(out, samples)
}

func (sp *StreamProcessor) Start() error {
    sp.mu.Lock()
    defer sp.mu.Unlock()
    
    if sp.isRunning {
        return nil
    }
    
    if err := sp.stream.Start(); err != nil {
        return fmt.Errorf("error starting stream: %v", err)
    }
    sp.isRunning = true
    return nil
}

func (sp *StreamProcessor) Stop() error {
    sp.mu.Lock()
    defer sp.mu.Unlock()
    
    if !sp.isRunning {
        return nil
    }
    
    if err := sp.stream.Stop(); err != nil {
        return fmt.Errorf("error stopping stream: %v", err)
    }
    sp.isRunning = false
    return nil
}

func (sp *StreamProcessor) Close() error {
    if err := sp.Stop(); err != nil {
        return err
    }
    return sp.stream.Close()
}