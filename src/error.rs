use std::backtrace::Backtrace;
use std::{error, fmt};

pub type Result<T> = std::result::Result<T, Error>;

pub trait ResultHelper<T>
{
	fn into_slhack(self) -> slhack::error::Result<T>;
}

impl<T> ResultHelper<T> for Result<T>
{
	fn into_slhack(self) -> slhack::error::Result<T>
	{
		self.map_err(Into::into)
	}
}

pub struct Error
{
	message: String,
	inner: Option<Box<dyn error::Error + 'static>>,
	backtrace: Backtrace,
}

impl Error
{
	pub fn new(message: String, inner: Option<Box<dyn error::Error + 'static>>) -> Self
	{
		Self {
			message: message,
			inner: inner,
			backtrace: Backtrace::capture(),
		}
	}

	pub fn from_parts(parts: (String, Option<Box<dyn error::Error + 'static>>)) -> Self
	{
		Self {
			message: parts.0,
			inner: parts.1,
			backtrace: Backtrace::capture(),
		}
	}

	pub fn into_parts(self) -> (String, Option<Box<dyn error::Error + 'static>>)
	{
		(self.message, self.inner)
	}

	pub fn context(self, message: String) -> Self
	{
		Error::new(message, Some(Box::new(self)))
	}
}

impl From<slhack::error::Error> for Error
{
	fn from(error: slhack::error::Error) -> Self
	{
		Self::from_parts(error.into_parts())
	}
}

impl Into<slhack::error::Error> for Error
{
	fn into(self: Error) -> slhack::error::Error
	{
		slhack::error::Error::from_parts(self.into_parts())
	}
}

impl From<String> for Error
{
	fn from(error: String) -> Self
	{
		Self {
			message: error,
			inner: None,
			backtrace: Backtrace::capture(),
		}
	}
}

impl From<hecs::NoSuchEntity> for Error
{
	fn from(error: hecs::NoSuchEntity) -> Self
	{
		Self {
			message: format!("{}", error),
			inner: Some(Box::new(error)),
			backtrace: Backtrace::capture(),
		}
	}
}

impl From<hecs::ComponentError> for Error
{
	fn from(error: hecs::ComponentError) -> Self
	{
		Self {
			message: format!("{}", error),
			inner: Some(Box::new(error)),
			backtrace: Backtrace::capture(),
		}
	}
}

impl fmt::Display for Error
{
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result
	{
		write!(f, "{}", self.message,)?;
		if let Some(ref inner) = self.inner
		{
			write!(f, "\nCause: {}", inner)?;
		}
		write!(f, "\nBacktrace:\n{}", self.backtrace)?;
		Ok(())
	}
}

impl error::Error for Error
{
	fn source(&self) -> Option<&(dyn error::Error + 'static)>
	{
		self.inner.as_ref().map(|e| &**e)
	}
}

impl fmt::Debug for Error
{
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result
	{
		write!(f, "{}", self)
	}
}
