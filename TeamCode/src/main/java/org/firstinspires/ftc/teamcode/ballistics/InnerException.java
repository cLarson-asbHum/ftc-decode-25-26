package ballistics;

class InnerException extends RuntimeException {
    InnerException(Exception cause) {
        super(cause);
    }
}