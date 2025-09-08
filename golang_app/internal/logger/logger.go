package logger

import (
	"log"

	"github.com/sirupsen/logrus"
)

func NewLogger(logLevel *string, name string) *logrus.Logger {
	// Parse log level
	level, err := logrus.ParseLevel(*logLevel)
	if err != nil {
		log.Fatalf("Invalid log level %s: %v", *logLevel, err)
	}

	// Create logger
	logger := logrus.New()
	logger.SetLevel(level)
	logger.SetFormatter(&logrus.TextFormatter{
		FullTimestamp: true,
		ForceColors:   true,
	})

	logger.WithField("component", name)

	return logger
}
